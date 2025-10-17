#include "composite_cbf/CbfSafetyFilter.hpp"
#include <ros/ros.h>


void CbfSafetyFilter::setAttVel(Eigen::Matrix3f& R_WB, Eigen::Vector3f& body_vel)
{
    double yaw = atan2(R_WB(1,0), R_WB(0,0));
    Eigen::Matrix3f R_WV = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    _R_BV = R_WB.transpose() * R_WV;

    _body_velocity = body_vel;
}

void CbfSafetyFilter::timeoutObstacles(double ts_now)
{
    if (_obstacles.size() && std::abs(_ts_obs - ts_now) > _to_obs)
    {
        ROS_WARN("[composite_cbf] obstacle timeout - clearing buffer");
        _obstacles.clear();
    }
}

void CbfSafetyFilter::setObstacles(std::vector<Eigen::Vector3f>& obstacles, double ts)
{
    _ts_obs = ts;
    _obstacles.clear();
    for (size_t i=0; i < obstacles.size(); ++i)
        _obstacles.push_back(obstacles[i]);
}

void CbfSafetyFilter::timeoutCmd(double ts_now)
{
    if (!_filtered_input.isZero() && std::abs(_ts_cmd - ts_now) > _to_cmd)
    {
        ROS_WARN("[composite_cbf] input cmd timeout - defaulting to (0,0,0)");
        _filtered_input.setZero();
    }
}

void CbfSafetyFilter::setCmd(Eigen::Vector3f& body_acceleration_setpoint, double ts)
{
    _ts_cmd = ts;
    _filtered_input = (1.f - _lp_gain_in) * _filtered_input + _lp_gain_in * body_acceleration_setpoint;
}

Eigen::Vector3f& CbfSafetyFilter::apply_filter(double ts_now)
{
    // timeout old inputs
    timeoutCmd(ts_now);
    timeoutObstacles(ts_now);

    // remove any nan obstacles
    _obstacles.erase(std::remove_if(_obstacles.begin(), _obstacles.end(),
        [](const Eigen::Vector3f& obs) { return obs.hasNaN(); }), _obstacles.end());

    // pass through if no obstacles are recorded
    const size_t n = _obstacles.size();
    if (n == 0)
        return _filtered_input;

    Eigen::Vector3f body_acc = _filtered_input;

    // composite collision CBF
    // nu1_i
    _nu1.clear();
    for(size_t i = 0; i < n; i++) {
        float nu_i0 = std::pow(_obstacles[i].norm(), 2) - (_epsilon * _epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_body_velocity);
        _nu1.push_back(Lf_nu_i0 - _pole0 * nu_i0);
        // std::cout << "nu1[" << i << "]: " << _nu1[i] << std::endl;
        // TODO compute tanh(nu1/gamma) only once and store to class array intead of nu1
    }

    // h(x)
    float exp_sum = 0.f;
    for(size_t i = 0; i < n; i++) {
        // if(std::isnan(_nu1[i]))
        //     continue;
        exp_sum += exp(-_kappa * saturate(_nu1[i] / _gamma));
    }
    float h = -(_gamma / _kappa) * logf(exp_sum);
    // std::cout << "h: " << h << " exp_sum" << exp_sum << std::endl;

    // L_{f}h(x)
    float Lf_h = 0.f;
    for(size_t i = 0; i < n; i++) {
        if(std::isnan(_nu1[i]))
            continue;
        float Lf_nu_i1 = 2.f * (_body_velocity + _pole0 * _obstacles[i]).dot(_body_velocity);
        float lambda_i = exp(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lf_h += lambda_i * Lf_nu_i1;
    }
    Lf_h /= exp_sum;

    // L_{g}h(x)z
    Eigen::Vector3f Lg_h(0.f, 0.f, 0.f);
    for(size_t i = 0; i < n; i++) {
        if(std::isnan(_nu1[i]))
            continue;
        Eigen::Vector3f Lg_nu_i1 = -2.f * _obstacles[i];
        float lambda_i = exp(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lg_h += lambda_i * Lg_nu_i1;
    }
    Lg_h /= exp_sum;

    // L_{g}h(x) * u, u = k_n(x) = a
    float Lg_h_u = Lg_h.dot(body_acc);

    // analytical QP solution from: https://arxiv.org/abs/2206.03568
    float eta = 0.f;
    float Lg_h_mag2 = std::pow(Lg_h.norm(), 2);
    if (Lg_h_mag2 > 1e-5f) {
        eta = -(Lf_h + Lg_h_u + _alpha*h) / Lg_h_mag2;
    }



    Eigen::Vector3f acceleration_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
    _unfiltered_ouput = body_acc + acceleration_correction;

    if(acceleration_correction.hasNaN())
        ROS_ERROR("[composite_cbf] NaN detected in acceleration correction: %f, %f, %f", acceleration_correction(0), acceleration_correction(1), acceleration_correction(2));

    // clamp and low pass acceleration output
    clampAccSetpoint(_unfiltered_ouput);
    _filtered_ouput = (1.f - _lp_gain_out) * _filtered_ouput + _lp_gain_out * _unfiltered_ouput;
    // check for NaN
    if(_filtered_ouput.hasNaN())
        ROS_ERROR("[composite_cbf] NaN detected in filtered output: %f, %f, %f", _filtered_ouput(0), _filtered_ouput(1), _filtered_ouput(2));
    if(_unfiltered_ouput.hasNaN())
        ROS_ERROR("[composite_cbf] NaN detected in unfiltered output: %f, %f, %f", _unfiltered_ouput(0), _unfiltered_ouput(1), _unfiltered_ouput(2));

    if (_filtered_ouput.hasNaN())
    {
        ROS_ERROR("[composite_cbf] NaN detected in QP solution - defaulting to (0,0,0)");
        _filtered_ouput.setZero();
    }

    return _filtered_ouput;
}

void CbfSafetyFilter::clampAccSetpoint(Eigen::Vector3f& acc) {
    acc(0) = std::min(std::max(acc(0), -_max_acc_xy), _max_acc_xy);
    acc(1) = std::min(std::max(acc(1), -_max_acc_xy), _max_acc_xy);
    acc(2) = std::min(std::max(acc(2), -_max_acc_z), _max_acc_z);
}

float CbfSafetyFilter::saturate(float x) {
    return std::tanh(x);
}

float CbfSafetyFilter::saturateDerivative(float x) {
    float th = std::tanh(x);
    return 1.f - (th * th);
}

float CbfSafetyFilter::kappaFunction(float h, float alpha) {
    float a = alpha;
    float b = 1.f / alpha;
    if (h >= 0.f) {
        return alpha * h;
    }
    else {
        return a * b * ( h / (b + std::abs(h)) );
    }
}
