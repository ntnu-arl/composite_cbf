#ifndef CBF_SAFETY_FILTER_HPP
#define CBF_SAFETY_FILTER_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <qpOASES.hpp>

using namespace qpOASES;

#define NV 5  // nb of QP variables
#define NC 5  // nb of QP consntraints
#define CBF_MAX_OBSTACLES 200
#define OBSTACLE_TIMEOUT_SEC 1  // 1 sec


class CbfSafetyFilter
{
public:
    CbfSafetyFilter() = default;
    ~CbfSafetyFilter() = default;


    void filter(Eigen::Vector3f& acceleration_setpoint);

    void setObstacles(std::vector<Eigen::Vector3f>& obstacles, double ts);
    void setAttVel(Eigen::Matrix3f& R_WB, Eigen::Vector3f& body_vel);

    void setFovH(float fov_h) { _fov_h = fov_h; }
    void setEpsilon(float epsilon) { _epsilon = epsilon; }
    void setPole0(float pole0) { _pole0 = pole0; }
    void setKappa(float kappa) { _kappa = kappa; }
    void setGamma(float gamma) { _gamma = gamma; }
    void setAlpha(float alpha) { _alpha = alpha; }
    void setFovAlpha(float fov_alpha) { _fov_alpha = fov_alpha; }
    // void setFovSlack(float fov_slack) { _fov_slack = fov_slack; }
    void setLpGainIn(float lp_gain_in) { _lp_gain_in = lp_gain_in; }
    void setLpGainOut(float lp_gain_out) { _lp_gain_out = lp_gain_out; }
    // void setQpGains(float gain_x, float gain_y, float gain_z) { _qp_gain_x = gain_x; _qp_gain_y = gain_y; _qp_gain_z = gain_z; }
    void setClampXY(float max_acc_xy) { _max_acc_xy = max_acc_xy; }
    void setClampZ(float max_acc_z) { _max_acc_z = max_acc_z; }
    void setAnalytical(bool analytical_sol) { _analytical_sol = analytical_sol; }

    Eigen::Matrix3f _R_BV;

private:
    void timeoutObstacles();

    double _ts_obs;
    std::vector<Eigen::Vector3f> _obstacles;
    Eigen::Vector3f _body_velocity;

    Eigen::Vector3f _filtered_input;
    Eigen::Vector3f _filtered_ouput;
    Eigen::Vector3f _unfiltered_ouput;
    std::vector<float> _nu1;

    float _fov_h;
    float _epsilon;
    float _pole0;
    float _kappa;
    float _gamma;
    float _alpha;
    float _fov_alpha;
    float _fov_slack;
    float _lp_gain_in;
    float _lp_gain_out;
    float _qp_gain_x;
    float _qp_gain_y;
    float _qp_gain_z;
    float _max_acc_xy;
    float _max_acc_z;
    bool _analytical_sol;

    void clampAccSetpoint(Eigen::Vector3f& acceleration_setpoint);
    float saturate(float x);
    float saturateDerivative(float x);
    float kappaFunction(float h, float alpha);
};

#endif // CBF_SAFETY_FILTER_HPP
