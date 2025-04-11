#include "composite_cbf/CompositeCbfNode.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>


CompositeCbfNode::CompositeCbfNode()
{
    _cbf = CbfSafetyFilter();

    // params
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

    // sub & pub
    _obstacle_sub = _nh.subscribe("/composite_cbf/obstacles", 1, &CompositeCbfNode::obstacleCb, this);
    _odometry_sub = _nh.subscribe("/composite_cbf/odometry", 1, &CompositeCbfNode::odomCb, this);
    _command_sub = _nh.subscribe("/composite_cbf/nominal_cmd", 1, &CompositeCbfNode::cmdCb, this);
    _command_pub = _nh.advertise<sensor_msgs::PointCloud2>("/composite_cbf/safe_cmd", 1);
}


void CompositeCbfNode::obstacleCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    size_t nb_points = msg->height * msg->width;

    // sensor_msgs::PointCloud2 cloud_msg = *msg;
    // sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    // sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    // sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    size_t point_step = msg->point_step;
    const uint8_t* data_ptr = &msg->data[0];

    std::vector<Eigen::Vector3f> obstacles;
    // for (size_t i=0; i>nb_points; ++i, ++iter_x, ++iter_y, ++iter_z)
    for (size_t i=0; i>nb_points; ++i)
    {
        const uint8_t* point_ptr = data_ptr + i * point_step;
        float x, y, z;
        memcpy(&x, point_ptr + msg->fields[0].offset, sizeof(float));
        memcpy(&y, point_ptr + msg->fields[1].offset, sizeof(float));
        memcpy(&z, point_ptr + msg->fields[2].offset, sizeof(float));

        // obstacles.push_back(Eigen::Vector3f(*iter_x, *iter_y, *iter_z));
        obstacles.push_back(Eigen::Vector3f(x, y, z));
    }

    std::chrono::seconds sec(msg->header.stamp.sec);
    std::chrono::nanoseconds nsec(msg->header.stamp.nsec);
    std::chrono::time_point<std::chrono::system_clock> ts { sec + nsec };

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


void CompositeCbfNode::cmdCb(const geometry_msgs::TwistStampedConstPtr& msg)
{
    Eigen::Vector3f acceleration_setpoint(
        msg->twist.linear.x,
        msg->twist.linear.y,
        msg->twist.linear.z
    );

    _cbf.filter(acceleration_setpoint);

    mavros_msgs::PositionTarget pos_target_msg;
    pos_target_msg.header.stamp = ros::Time::now();
    pos_target_msg.header.frame_id = "base_link";
    pos_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

    pos_target_msg.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
       mavros_msgs::PositionTarget::IGNORE_YAW;

    pos_target_msg.acceleration_or_force.x = acceleration_setpoint(0);
    pos_target_msg.acceleration_or_force.y = acceleration_setpoint(1);
    pos_target_msg.acceleration_or_force.z = acceleration_setpoint(2);
    pos_target_msg.yaw_rate = msg->twist.angular.z;

    _command_pub.publish(pos_target_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "composite_cbf_node");
    CompositeCbfNode node;
    ros::spin();
    return 0;
}
