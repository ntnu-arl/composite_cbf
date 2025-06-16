#ifndef CBF_SAFETY_FILTER_HPP
#define CBF_SAFETY_FILTER_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>


class CbfSafetyFilter
{
public:
    CbfSafetyFilter() = default;
    ~CbfSafetyFilter() = default;

    Eigen::Vector3f& apply_filter(double ts_now);

    void setCmd(Eigen::Vector3f& body_acceleration_setpoint, double ts);
    void setObstacles(std::vector<Eigen::Vector3f>& obstacles, double ts);
    void setAttVel(Eigen::Matrix3f& R_WB, Eigen::Vector3f& body_vel);

    void setObsTo(float to) { _to_obs = to; }
    void setCmdTo(float to) { _to_cmd = to; }
    void setEpsilon(float epsilon) { _epsilon = epsilon; }
    void setPole0(float pole0) { _pole0 = pole0; }
    void setKappa(float kappa) { _kappa = kappa; }
    void setGamma(float gamma) { _gamma = gamma; }
    void setAlpha(float alpha) { _alpha = alpha; }
    void setLpGainIn(float lp_gain_in) { _lp_gain_in = lp_gain_in; }
    void setLpGainOut(float lp_gain_out) { _lp_gain_out = lp_gain_out; }
    void setClampXY(float max_acc_xy) { _max_acc_xy = max_acc_xy; }
    void setClampZ(float max_acc_z) { _max_acc_z = max_acc_z; }

    Eigen::Matrix3f _R_BV;

private:
    void timeoutObstacles(double ts_now);
    void timeoutCmd(double ts_now);

    double _ts_obs, _to_obs;
    double _ts_cmd, _to_cmd;
    std::vector<Eigen::Vector3f> _obstacles;
    Eigen::Vector3f _body_velocity;

    Eigen::Vector3f _filtered_input = Eigen::Vector3f(0,0,0);
    Eigen::Vector3f _unfiltered_ouput = Eigen::Vector3f(0,0,0);
    Eigen::Vector3f _filtered_ouput = Eigen::Vector3f(0,0,0);

    std::vector<float> _nu1;

    float _epsilon;
    float _pole0;
    float _kappa;
    float _gamma;
    float _alpha;
    float _lp_gain_in;
    float _lp_gain_out;
    float _max_acc_xy;
    float _max_acc_z;

    void clampAccSetpoint(Eigen::Vector3f& acceleration_setpoint);
    float saturate(float x);
    float saturateDerivative(float x);
    float kappaFunction(float h, float alpha);
};

#endif // CBF_SAFETY_FILTER_HPP
