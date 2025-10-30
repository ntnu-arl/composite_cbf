#include "composite_cbf/CbfSafetyFilter.hpp"


uint32_t CbfSafetyFilter::popStatus() {
    uint32_t tmp = _status;
    _status = CBF_OK;
    return tmp;
}

void CbfSafetyFilter::setStatus(uint32_t flag) {
    _status |= flag;
}

void CbfSafetyFilter::timeoutObstacles(double ts_now)
{
    if (_obstacles.size() && std::abs(_ts_obs - ts_now) > _cfg.obs_to)
    {
        _obstacles.clear();
        setStatus(CBF_WARN_OBS_TIMEOUT);
    }
}

void CbfSafetyFilter::setObstacles(std::vector<Eigen::Vector3f>& obstacles, double ts)
{
    _ts_obs = ts;
    _obstacles = obstacles;
}

void CbfSafetyFilter::timeoutCmd(double ts_now)
{
    if (!_filtered_input.isZero() && std::abs(_ts_cmd - ts_now) > _cfg.cmd_to)
    {
        _filtered_input.setZero();
        setStatus(CBF_WARN_CMD_TIMEOUT);
    }
}

void CbfSafetyFilter::setCmd(Eigen::Vector3f& body_acceleration_setpoint, double ts)
{
    _ts_cmd = ts;
    _filtered_input = (1.f - _cfg.lp_gain_in) * _filtered_input + _cfg.lp_gain_in * body_acceleration_setpoint;
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

    Eigen::Vector3f body_acc = _filtered_input;

    // composite collision CBF
    // nu1_i
    _nu1_gamma.clear();
    for (size_t i = 0; i < n; i++)
    {
        float nu_i0 = std::pow(_obstacles[i].norm(), 2) - (_cfg.epsilon * _cfg.epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_body_velocity);
        _nu1_gamma.push_back((Lf_nu_i0 - _cfg.pole_0 * nu_i0) / _cfg.gamma);
        // TODO compute tanh(nu1/gamma) only once and store to class array intead of nu1
    }

    // h(x)
    float exp_sum = 0.f;
    for (size_t i = 0; i < n; i++)
        exp_sum += exp(-_cfg.kappa * saturate(_nu1_gamma[i]));
    float h = -(_cfg.gamma / _cfg.kappa) * logf(exp_sum);

    // L_{f}h(x)
    float Lf_h = 0.f;
    for (size_t i = 0; i < n; i++)
    {
        float Lf_nu_i1 = 2.f * (_body_velocity + _cfg.pole_0 * _obstacles[i]).dot(_body_velocity);
        float lambda_i = exp(-_cfg.kappa * saturate(_nu1_gamma[i])) * saturateDerivative(_nu1_gamma[i]);
        Lf_h += lambda_i * Lf_nu_i1;
    }
    Lf_h /= exp_sum;

    // L_{g}h(x)z
    Eigen::Vector3f Lg_h(0.f, 0.f, 0.f);
    for (size_t i = 0; i < n; i++)
    {
        Eigen::Vector3f Lg_nu_i1 = -2.f * _obstacles[i];
        float lambda_i = exp(-_cfg.kappa * saturate(_nu1_gamma[i])) * saturateDerivative(_nu1_gamma[i]);
        Lg_h += lambda_i * Lg_nu_i1;
    }
    Lg_h /= exp_sum;

    // L_{g}h(x) * u, u = k_n(x) = a
    float Lg_h_u = Lg_h.dot(body_acc);

    // analytical QP solution from: https://arxiv.org/abs/2206.03568
    float eta = 0.f;
    float Lg_h_mag2 = std::pow(Lg_h.norm(), 2);
    if (Lg_h_mag2 > 1e-5f)
        eta = -(Lf_h + Lg_h_u + _cfg.alpha*h) / Lg_h_mag2;

    Eigen::Vector3f acceleration_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
    Eigen::Vector3f unfiltered_ouput = body_acc + acceleration_correction;

    // clamp and low pass acceleration output
    clampAccSetpoint(unfiltered_ouput);
    _filtered_ouput = (1.f - _cfg.lp_gain_out) * _filtered_ouput + _cfg.lp_gain_out * unfiltered_ouput;

    if (_filtered_ouput.hasNaN())
    {
        _filtered_ouput.setZero();
        setStatus(CBF_ERR_NAN_OUTPUT);
    }

    return _filtered_ouput;
}

void CbfSafetyFilter::clampAccSetpoint(Eigen::Vector3f& acc) {
    acc(0) = std::min(std::max(acc(0), -_cfg.max_acc_xy), _cfg.max_acc_xy);
    acc(1) = std::min(std::max(acc(1), -_cfg.max_acc_xy), _cfg.max_acc_xy);
    acc(2) = std::min(std::max(acc(2), -_cfg.max_acc_z), _cfg.max_acc_z);
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
