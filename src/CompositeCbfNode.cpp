#include "composite_cbf/CompositeCbfNode.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>


CompositeCbfNode::CompositeCbfNode()
{
    _cbf = CbfSafetyFilter();

    // params
    _nh.getParam("/composite_cbf/output_frame_viz", _frame_body);
    _nh.getParam("/composite_cbf/ctrl_freq", _ctrl_freq);
    float fov_h;
    _nh.getParam("/composite_cbf/fov_h", fov_h);
    _cbf.setFovH(fov_h);
    float epsilon;
    _nh.getParam("/composite_cbf/epsilon", epsilon);
    _cbf.setEpsilon(epsilon);
    float pole_0;
    _nh.getParam("/composite_cbf/pole_0", pole_0);
    _cbf.setPole0(pole_0);
    float kappa;
    _nh.getParam("/composite_cbf/kappa", kappa);
    _cbf.setKappa(kappa);
    float gamma;
    _nh.getParam("/composite_cbf/gamma", gamma);
    _cbf.setGamma(gamma);
    float alpha;
    _nh.getParam("/composite_cbf/alpha", alpha);
    _cbf.setAlpha(alpha);
    float fov_alpha;
    _nh.getParam("/composite_cbf/fov_alpha", fov_alpha);
    _cbf.setFovAlpha(fov_alpha);
    float fov_slack;
    _nh.getParam("/composite_cbf/fov_slack", fov_slack);
    _cbf.setFovSlack(fov_slack);
    float lp_gain_in;
    _nh.getParam("/composite_cbf/lp_gain_in", lp_gain_in);
    _cbf.setLpGainIn(lp_gain_in);
    float lp_gain_out;
    _nh.getParam("/composite_cbf/lp_gain_out", lp_gain_out);
    _cbf.setLpGainOut(lp_gain_out);
    float clamp_xy;
    _nh.getParam("/composite_cbf/clamp_xy", clamp_xy);
    _cbf.setClampXY(clamp_xy);
    float clamp_z;
    _nh.getParam("/composite_cbf/clamp_z", clamp_z);
    _cbf.setClampZ(clamp_z);
    float gain_x, gain_y, gain_z;
    _nh.getParam("/composite_cbf/gain_x", gain_x);
    _nh.getParam("/composite_cbf/gain_y", gain_y);
    _nh.getParam("/composite_cbf/gain_z", gain_z);
    _cbf.setQpGains(gain_x, gain_y, gain_z);
    bool analytical_sol;
    _nh.getParam("/composite_cbf/analytical_sol", analytical_sol);
    _cbf.setAnalytical(analytical_sol);

    // sub & pub
    _obstacle_sub = _nh.subscribe("/composite_cbf/obstacles", 1, &CompositeCbfNode::obstacleCb, this);
    _odometry_sub = _nh.subscribe("/composite_cbf/odometry", 1, &CompositeCbfNode::odomCb, this);
    _command_sub = _nh.subscribe("/composite_cbf/nominal_cmd", 1, &CompositeCbfNode::cmdCb, this);
    _command_pub_twist = _nh.advertise<geometry_msgs::Twist>("/composite_cbf/safe_cmd_twist", 1);
    _command_pub_postarget = _nh.advertise<mavros_msgs::PositionTarget>("/composite_cbf/safe_cmd_postarget", 1);
    _output_viz_pub = _nh.advertise<geometry_msgs::TwistStamped>("/composite_cbf/output_viz", 1);
    _input_viz_pub = _nh.advertise<geometry_msgs::TwistStamped>("/composite_cbf/input_viz", 1);

    // timer
    float period = 1.f / (float) _ctrl_freq;
    _cmd_timer = _nh.createTimer(ros::Duration(period), &CompositeCbfNode::ctrlCb, this);
}


void CompositeCbfNode::ctrlCb(const ros::TimerEvent& event) {
    // cbf filtering
    double now = ros::Time::now().toSec();
    Eigen::Vector3f acc_sp = _cbf.apply_filter(now);

    // publish mavros msg
    mavros_msgs::PositionTarget msg_safe_postarget;
    msg_safe_postarget.header.stamp = ros::Time::now();
    msg_safe_postarget.header.frame_id = _frame_body;
    msg_safe_postarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    msg_safe_postarget.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_YAW;
    msg_safe_postarget.acceleration_or_force.x = acc_sp(0);
    msg_safe_postarget.acceleration_or_force.y = acc_sp(1);
    msg_safe_postarget.acceleration_or_force.z = acc_sp(2);
    msg_safe_postarget.yaw_rate = _wz_des;
    _command_pub_postarget.publish(msg_safe_postarget);

    // publish twist msg
    geometry_msgs::Twist msg_safe_twist;
    msg_safe_twist.linear.x = acc_sp(0);
    msg_safe_twist.linear.y = acc_sp(1);
    msg_safe_twist.linear.z = acc_sp(2);
    msg_safe_twist.angular.z = _wz_des;
    _command_pub_twist.publish(msg_safe_twist);

    // publish viz output msg
    geometry_msgs::TwistStamped msg_viz_out;
    msg_viz_out.header.stamp = ros::Time::now();
    msg_viz_out.header.frame_id = _frame_body;
    msg_viz_out.twist = msg_safe_twist;
    _output_viz_pub.publish(msg_viz_out);
}


void CompositeCbfNode::obstacleCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    size_t nb_points = msg->height * msg->width;

    size_t point_step = msg->point_step;
    const uint8_t* data_ptr = &msg->data[0];

    std::vector<Eigen::Vector3f> obstacles;
    for (size_t i=0; i<nb_points; ++i)
    {
        const uint8_t* point_ptr = data_ptr + i * point_step;
        float x, y, z;
        memcpy(&x, point_ptr + msg->fields[0].offset, sizeof(float));
        memcpy(&y, point_ptr + msg->fields[1].offset, sizeof(float));
        memcpy(&z, point_ptr + msg->fields[2].offset, sizeof(float));

        obstacles.push_back(Eigen::Vector3f(x, y, z));
    }

    double ts = msg->header.stamp.toSec();
    _cbf.setObstacles(obstacles, ts);
}


void CompositeCbfNode::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    Eigen::Quaternionf eigen_quat(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    Eigen::Matrix3f rotation_matrix = eigen_quat.toRotationMatrix();

    Eigen::Vector3f body_velocity(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );

    _cbf.setAttVel(rotation_matrix, body_velocity);
}


void CompositeCbfNode::cmdCb(const geometry_msgs::TwistConstPtr& msg)
{
    Eigen::Vector3f acceleration_setpoint(
        msg->linear.x,
        msg->linear.y,
        msg->linear.z
    );
    acceleration_setpoint = _cbf._R_BV * acceleration_setpoint;  // TODO for now joystick input is actually vehicle frame

    double ts = ros::Time::now().toSec();
    _cbf.setCmd(acceleration_setpoint, ts);
    _wz_des = msg->angular.z;

    // publish viz input msg
    geometry_msgs::TwistStamped msg_viz_in;
    msg_viz_in.header.stamp = ros::Time::now();
    msg_viz_in.header.frame_id = _frame_body;
    msg_viz_in.twist.linear.x = acceleration_setpoint(0);
    msg_viz_in.twist.linear.y = acceleration_setpoint(1);
    msg_viz_in.twist.linear.z = acceleration_setpoint(2);
    _input_viz_pub.publish(msg_viz_in);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "composite_cbf_node");
    CompositeCbfNode node;
    ros::spin();
    return 0;
}
