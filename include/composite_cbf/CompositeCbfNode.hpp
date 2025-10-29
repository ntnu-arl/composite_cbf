#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include "composite_cbf/CbfSafetyFilter.hpp"


class CompositeCbfNode : public rclcpp::Node
{
public:
    CompositeCbfNode();

private:
    void obstacleCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odometryCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void commandCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdTimerCb();

    CbfSafetyFilter _cbf;

    std::string _frame_body;
    float _ctrl_freq;
    float _wz_des;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _obstacle_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _command_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _command_pub_twist;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr _command_pub_postarget;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _output_viz_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _input_viz_pub;
    rclcpp::TimerBase::SharedPtr _cmd_timer;
};