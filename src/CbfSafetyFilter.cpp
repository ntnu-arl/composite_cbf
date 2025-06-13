#include "composite_cbf/CbfSafetyFilter.hpp"
#include <ros/ros.h>
#include <iostream>

void CbfSafetyFilter::setAttVel(Eigen::Matrix3f& R_WB, Eigen::Vector3f& body_vel)
{
    double yaw = atan2(R_WB(1,0), R_WB(0,0));
    Eigen::Matrix3f R_WV = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    _R_BV = R_WB.transpose() * R_WV;

    _body_velocity = body_vel;
}

void CbfSafetyFilter::timeoutObstacles(double ts_now)
{
    if (_obstacles.size() && std::abs(_ts_obs - ts_now) > OBSTACLE_TIMEOUT_SEC)
    {
        ROS_WARN("[composite_cbf] obstacle timeout - clearing buffer");
        _obstacles.clear();
    }
}

void CbfSafetyFilter::setObstacles(std::vector<Eigen::Vector3f>& obstacles, double ts)
{
    _ts_obs = ts;
    _obstacles.clear();
    for (int i=0; i < std::min((int)obstacles.size(), CBF_MAX_OBSTACLES); ++i)
        _obstacles.push_back(obstacles[i]);
}

void CbfSafetyFilter::timeoutCmd(double ts_now)
{
    if (!_filtered_input.isZero() && std::abs(_ts_cmd - ts_now) > OBSTACLE_TIMEOUT_SEC)
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

    // pass through if no obstacles are recorded
    const size_t n = _obstacles.size();
    if (n == 0)
        return _filtered_input;

    // composite collision CBF
    // nu1_i
    _nu1.clear();
    for(size_t i = 0; i < n; i++) {
        float nu_i0 = std::pow(_obstacles[i].norm(), 2) - (_epsilon * _epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_body_velocity);
        _nu1.push_back(Lf_nu_i0 - _pole0 * nu_i0);
        // TODO compute tanh(nu1/gamma) only once and store to class array intead of nu1
    }

    // h(x)
    float exp_sum = 0.f;
    for(size_t i = 0; i < n; i++) {
        exp_sum += exp(-_kappa * saturate(_nu1[i] / _gamma));
    }
    float h = -(_gamma / _kappa) * logf(exp_sum);

    // L_{f}h(x)
    float Lf_h = 0.f;
    for(size_t i = 0; i < n; i++) {
        float Lf_nu_i1 = 2.f * (_body_velocity + _pole0 * _obstacles[i]).dot(_body_velocity);
        float lambda_i = exp(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lf_h += lambda_i * Lf_nu_i1;
    }
    Lf_h /= exp_sum;

    // L_{g}h(x)z
    Eigen::Vector3f Lg_h(0.f, 0.f, 0.f);
    for(size_t i = 0; i < n; i++) {
        Eigen::Vector3f Lg_nu_i1 = -2.f * _obstacles[i];
        float lambda_i = exp(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lg_h += lambda_i * Lg_nu_i1;
    }
    Lg_h /= exp_sum;

    // L_{g}h(x) * u, u = k_n(x) = a
    float Lg_h_u = Lg_h.dot(_filtered_input);

    // horizontal FoV CBF
    Eigen::Vector3f e1(sin(_fov_h), cos(_fov_h), 0.f);
    Eigen::Vector3f e2(sin(_fov_h), -cos(_fov_h), 0.f);
    e1 = _R_BV * e1;
    e2 = _R_BV * e2;
    float h1 = (e1).dot(_body_velocity);
    float h2 = (e2).dot(_body_velocity);
    float Lf_h1 = 0.f;
    float Lf_h2 = 0.f;
    Eigen::Vector3f Lg_h1 = e1;
    Eigen::Vector3f Lg_h2 = e2;
    // uncomment below to disable fov constraints TODO make this an argument
    // Lg_h1 = Eigen::Vector3f(0,0,0);
    // Lg_h2 = Eigen::Vector3f(0,0,0);

    Eigen::Vector3f acceleration_correction(0,0,0);

    if (_analytical_sol)
    {
        // analytical QP solution from: https://arxiv.org/abs/2206.03568
        float eta = 0.f;
        float Lg_h_mag2 = std::pow(Lg_h.norm(), 2);
        if (Lg_h_mag2 > 1e-5f) {
            eta = -(Lf_h + Lg_h_u + _alpha*h) / Lg_h_mag2;
        }

        acceleration_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
        _unfiltered_ouput = _filtered_input + acceleration_correction;

        // ========================
        // ========================
        // ========================
        // implement the FoV constraints. This is done in a sub-optima fashoin by projecting the acceleration after the collision filtering onto the feasible subspace.
        float violation1 = Lf_h1 + Lg_h1.dot(_unfiltered_ouput) + _fov_alpha * h1;
        float violation2 = Lf_h2 + Lg_h2.dot(_unfiltered_ouput) + _fov_alpha * h2;


        // Project _unfiltered_output to make both violations non-negative
        // TODO: This simplified version assumes the FoV is 90 degrees or larger
        if (violation1 < 0.f || violation2 < 0.f) {
            if (violation1 < 0.f && violation2 >= 0.f) {
                // // Solve for the first constraint
                // Eigen::Matrix<float, 1, 3> A1;
                // A1.setRow(0, Lg_h1);
                // Eigen::Vector<float, 1> b1;
                // b1(0) = -violation1;

                // Analytical solution for single constraint: delta = -(Ax-b)A^T/(AA^T)
                Eigen::Vector3f A1_T = Lg_h1;
                float A1_A1T = Lg_h1.dot(Lg_h1);
                Eigen::Vector3f delta1 = (A1_A1T > 1e-6f) ? (-violation1 * A1_T / A1_A1T) : Eigen::Vector3f();

                _unfiltered_ouput += delta1;

            } else if (violation2 < 0.f && violation1 >= 0.f) {
                // // Solve for the second constraint
                // Eigen::Matrix<float, 1, 3> A2;
                // A2.setRow(0, Lg_h2);
                // Eigen::Vector<float, 1> b2;
                // b2(0) = -violation2;

                // Analytical solution for single constraint: delta = -(Ax-b)A^T/(AA^T)
                Eigen::Vector3f A2_T = Lg_h2;
                float A2_A2T = Lg_h2.dot(Lg_h2);
                Eigen::Vector3f delta2 = (A2_A2T > 1e-6f) ? (-violation2 * A2_T / A2_A2T) : Eigen::Vector3f();

                _unfiltered_ouput += delta2;

            } else {
                // Solve for both constraints
                Eigen::Matrix<float, 2, 3> A;
                A(0,0) = Lg_h1(0);
                A(0,1) = Lg_h1(1);
                A(0,2) = Lg_h1(2);
                A(1,0) = Lg_h2(0);
                A(1,1) = Lg_h2(1);
                A(1,2) = Lg_h2(2);
                Eigen::Vector2f b;
                b(0) = -violation1;
                b(1) = -violation2;

                // Direct analytical solution for two constraints using A^T(AA^T)^(-1)b
                Eigen::Matrix2f AAT = A * A.transpose();
                float det = AAT(0,0)*AAT(1,1) - AAT(0,1)*AAT(1,0);

                Eigen::Matrix2f AAT_inv;
                AAT_inv(0,0) = AAT(1,1)/det;
                AAT_inv(0,1) = -AAT(0,1)/det;
                AAT_inv(1,0) = -AAT(1,0)/det;
                AAT_inv(1,1) = AAT(0,0)/det;
                Eigen::Vector3f delta = A.transpose() * (AAT_inv * b);

                _unfiltered_ouput += delta;
            }
        }
        // ========================
        // ========================
        // ========================
    }
    else
    {
        ROS_ERROR("[composite_cbf] QP soltion is currently not working with FoV constraints! no filter performed");
        return _filtered_input;

        // solve QP
        // quadratic cost x^T*H*x
        real_t H[NV*NV] = {(real_t)_qp_gain_x, 0.0, 0.0, 0.0, 0.0,
                           0.0, (real_t)_qp_gain_y, 0.0, 0.0, 0.0,
                           0.0, 0.0, (real_t)_qp_gain_z, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0};
        // linear cost matrix g*x
        real_t  g[NV] = { 0.0, 0.0, 0.0, (real_t)_fov_slack, (real_t)_fov_slack };
        // constraint matrix A
        real_t  A[NC*NV] = {(real_t)Lg_h(0),  (real_t)Lg_h(1),  (real_t)Lg_h(2),  0.0, 0.0,
                            0.0,              0.0,              0.0,              1.0, 0.0,
                            (real_t)Lg_h1(0), (real_t)Lg_h1(1), (real_t)Lg_h1(2), 1.0, 0.0,
                            0.0,              0.0,              0.0,              0.0, 1.0,
                            (real_t)Lg_h2(0), (real_t)Lg_h2(1), (real_t)Lg_h2(2), 0.0, 1.0};
        // bounds on Ax
        real_t  lbA[NC] = { (real_t)(-Lf_h - kappaFunction(h, _alpha) - Lg_h_u), 0.0, (real_t)(-Lf_h1 - _fov_alpha * h1), 0.0, (real_t)(-Lf_h2 - _fov_alpha * h2) };
        real_t* ubA = NULL;
        // bounds on x
        real_t* lb = NULL;
        real_t* ub = NULL;
        int_t nWSR = 50;

        QProblem qp(NV, NC);
        qp.setPrintLevel(PL_NONE);
        returnValue qp_status = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

        real_t xOpt[NV];
        switch(qp_status) {
            case SUCCESSFUL_RETURN:
            {
                qp.getPrimalSolution(xOpt);
                acceleration_correction = Eigen::Vector3f(xOpt[0], xOpt[1], xOpt[2]);
                break;
            }
            case RET_MAX_NWSR_REACHED:
            {
                ROS_ERROR("[composite_cbf] QP could not be solved within the given number of working set recalculations");
                break;
            }
            default:
            {
                ROS_ERROR_STREAM("[composite_cbf] QP failed: returned " << qp_status);
                break;
            }
        }
    }

    // clamp and low pass acceleration output
    _unfiltered_ouput = _filtered_input + acceleration_correction;
    clampAccSetpoint(_unfiltered_ouput);
    _filtered_ouput = (1.f - _lp_gain_out) * _filtered_ouput + _lp_gain_out * _unfiltered_ouput;

    if (_filtered_ouput.hasNaN())
    {
        ROS_ERROR("[composite_cbf] NaN detected is QP solution - defaulting to (0,0,0)");
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
