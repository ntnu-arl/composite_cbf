#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Dense>


enum CbfStatus : uint32_t {
    CBF_OK               = 0,
    CBF_WARN_OBS_TIMEOUT = 1u << 0,
    CBF_WARN_CMD_TIMEOUT = 1u << 1,
    CBF_ERR_NAN_OUTPUT   = 1u << 2,
};

struct CbfConfig {
    float epsilon = 0.7f;
    float pole_0 = -2.5f;
    float kappa = 70.f;
    float gamma = 40.f;
    float alpha = 2.f;
    float lp_gain_in = 0.4f;
    float lp_gain_out = 0.6f;
    float max_acc_xy = 3.f;
    float max_acc_z = 3.f;
    float obs_to = 1.f;
    float cmd_to = 1.f;
};

class CbfSafetyFilter
{
public:
    CbfSafetyFilter() = default;
    ~CbfSafetyFilter() = default;

    void setConfig(const CbfConfig& c) { _cfg = c; }
    const CbfConfig& config() const { return _cfg; }

    Eigen::Vector3f& apply_filter(double ts_now);

    void setCmd(Eigen::Vector3f& body_acceleration_setpoint, double ts);
    void setObstacles(std::vector<Eigen::Vector3f>& obstacles, double ts);
    void setVel(Eigen::Vector3f& body_vel) { _body_velocity = body_vel; };

    // pop (get and reset) status
    uint32_t popStatus();

private:
    void timeoutObstacles(double ts_now);
    void timeoutCmd(double ts_now);

    double _ts_obs;
    double _ts_cmd;
    std::vector<Eigen::Vector3f> _obstacles;
    Eigen::Vector3f _body_velocity;

    Eigen::Vector3f _filtered_input = Eigen::Vector3f(0,0,0);
    Eigen::Vector3f _filtered_ouput = Eigen::Vector3f(0,0,0);

    CbfConfig _cfg;

    std::vector<float> _nu1_gamma;  // stored in class to avoid reallocating at each filter loop

    uint32_t _status{CBF_OK};
    void setStatus(uint32_t flag);

    void clampAccSetpoint(Eigen::Vector3f& acceleration_setpoint);
    float saturate(float x);
    float saturateDerivative(float x);
    float kappaFunction(float h, float alpha);
};
