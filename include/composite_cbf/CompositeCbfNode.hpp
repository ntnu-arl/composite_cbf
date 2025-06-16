#ifndef COMPOSITE_CBF_HPP
#define COMPOSITE_CBF_HPP


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include "composite_cbf/CbfSafetyFilter.hpp"


class CompositeCbfNode
{
public:
    CompositeCbfNode();

private:
    void obstacleCb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCb(const nav_msgs::OdometryConstPtr& msg);
    void cmdCb(const geometry_msgs::TwistConstPtr& msg);
    void ctrlCb(const ros::TimerEvent& event);

    CbfSafetyFilter _cbf;

    std::string _frame_body;
    int _ctrl_freq;
    float _wz_des;

    ros::NodeHandle _nh;

    ros::Subscriber _obstacle_sub;
    ros::Subscriber _odometry_sub;
    ros::Subscriber _command_sub;
    ros::Publisher _command_pub_twist;
    ros::Publisher _command_pub_postarget;
    ros::Publisher _output_viz_pub;
    ros::Publisher _input_viz_pub;
    ros::Timer _cmd_timer;
};

#endif // COMPOSITE_CBF_HPP
