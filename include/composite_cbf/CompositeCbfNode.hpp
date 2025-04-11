#ifndef COMPOSITE_CBF_HPP
#define COMPOSITE_CBF_HPP


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
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
    void cmdCb(const geometry_msgs::TwistStampedConstPtr& msg);

    CbfSafetyFilter _cbf;

    ros::NodeHandle _nh;

    ros::Subscriber _obstacle_sub;
    ros::Subscriber _odometry_sub;
    ros::Subscriber _command_sub;
    ros::Publisher _command_pub;
};

#endif // COMPOSITE_CBF_HPP
