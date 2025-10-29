#include "composite_cbf/CompositeCbfNode.hpp"


using std::placeholders::_1;

CompositeCbfNode::CompositeCbfNode()
: rclcpp::Node("composite_cbf_node")
{
    _cbf = CbfSafetyFilter();

    // load config
    this->declare_parameter<std::string>("output_frame_viz");
    this->declare_parameter<float>("ctrl_freq");
    this->declare_parameter<float>("epsilon");
    this->declare_parameter<float>("pole_0");
    this->declare_parameter<float>("kappa");
    this->declare_parameter<float>("gamma");
    this->declare_parameter<float>("alpha");
    this->declare_parameter<float>("lp_gain_in");
    this->declare_parameter<float>("lp_gain_out");
    this->declare_parameter<float>("max_acc_xy");
    this->declare_parameter<float>("clamp_z");
    this->declare_parameter<float>("obs_to");
    this->declare_parameter<float>("cmd_to");

    this->get_parameter("output_frame_viz");
    this->get_parameter("ctrl_freq", _ctrl_freq);

    CbfConfig cfg;
    this->get_parameter("epsilon", cfg.epsilon);
    this->get_parameter("pole_0", cfg.pole_0);
    this->get_parameter("kappa", cfg.kappa);
    this->get_parameter("gamma", cfg.gamma);
    this->get_parameter("alpha", cfg.alpha);
    this->get_parameter("lp_gain_in", cfg.lp_gain_in);
    this->get_parameter("lp_gain_out", cfg.lp_gain_out);
    this->get_parameter("max_acc_xy", cfg.max_acc_xy);
    this->get_parameter("max_acc_z", cfg.max_acc_z);
    this->get_parameter("obs_to", cfg.obs_to);
    this->get_parameter("cmd_to", cfg.cmd_to);
    _cbf.setConfig(cfg);
    
    // sub & pub
    _command_pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("/composite_cbf/safe_cmd_twist", 10);
    _command_pub_postarget = this->create_publisher<mavros_msgs::msg::PositionTarget>("/composite_cbf/safe_cmd_postarget", 10);
    _output_viz_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("composite_cbf/output_viz", 10);
    _input_viz_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("composite_cbf/input_viz", 10);

    _obstacle_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "obstacles", rclcpp::SensorDataQoS(),
    std::bind(&CompositeCbfNode::obstacleCb, this, _1));
    _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&CompositeCbfNode::odometryCb, this, _1));
    _command_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_in", 10, std::bind(&CompositeCbfNode::commandCb, this, _1));

    // timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.f, _ctrl_freq));

    _cmd_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CompositeCbfNode::cmdTimerCb, this)
    );
}

void CompositeCbfNode::obstacleCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

    double now = rclcpp::Time(msg->header.stamp).seconds();
    _cbf.setObstacles(obstacles, now);
}

void CompositeCbfNode::odometryCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Vector3f body_velocity(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );

    _cbf.setVel(body_velocity);
}

void CompositeCbfNode::commandCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    Eigen::Vector3f acceleration_setpoint(
        msg->linear.x,
        msg->linear.y,
        msg->linear.z
    );  // assumed to be body frame
    double now = rclcpp::Time(this->now()).seconds();
    _cbf.setCmd(acceleration_setpoint, now);

    _wz_des = msg->angular.z;

    // publish viz input msg
    geometry_msgs::msg::TwistStamped msg_viz_in;
    msg_viz_in.header.stamp = this->now();
    msg_viz_in.header.frame_id = _frame_body;
    msg_viz_in.twist.linear.x = acceleration_setpoint(0);
    msg_viz_in.twist.linear.y = acceleration_setpoint(1);
    msg_viz_in.twist.linear.z = acceleration_setpoint(2);
    _input_viz_pub->publish(msg_viz_in);
}

void CompositeCbfNode::cmdTimerCb()
{
    // cbf filtering
    double now = rclcpp::Time(this->now()).seconds();
    Eigen::Vector3f acc_sp = _cbf.apply_filter(now);

    // check status
    uint32_t status = _cbf.popStatus();  // this resets the status
    if (status & CBF_WARN_OBS_TIMEOUT)
        RCLCPP_WARN(get_logger(), "obstacle timeout - clearing buffer");
    if (status & CBF_WARN_CMD_TIMEOUT)
        RCLCPP_WARN(get_logger(), "input cmd timeout - defaulting to (0,0,0)");
    if (status & CBF_ERR_NAN_OUTPUT)
        RCLCPP_ERROR(get_logger(), "NaN detected in QP solution - defaulting to (0,0,0)");

    // publish twist msg
    geometry_msgs::msg::Twist msg_safe_twist{};
    msg_safe_twist.linear.x = acc_sp(0);
    msg_safe_twist.linear.y = acc_sp(1);
    msg_safe_twist.linear.z = acc_sp(2);
    msg_safe_twist.angular.z = _wz_des;
    _command_pub_twist->publish(msg_safe_twist);

    // publish viz output msg
    geometry_msgs::msg::TwistStamped msg_viz_out;
    msg_viz_out.header.stamp = this->now();
    msg_viz_out.header.frame_id = _frame_body;
    msg_viz_out.twist = msg_safe_twist;
    _output_viz_pub->publish(msg_viz_out);

    // publish mavros msg
    mavros_msgs::msg::PositionTarget msg_safe_postarget;
    msg_safe_postarget.header.stamp = msg_viz_out.header.stamp;
    msg_safe_postarget.header.frame_id = _frame_body;
    msg_safe_postarget.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
    msg_safe_postarget.type_mask = 
        mavros_msgs::msg::PositionTarget::IGNORE_PX |
        mavros_msgs::msg::PositionTarget::IGNORE_PY |
        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
        mavros_msgs::msg::PositionTarget::IGNORE_VX |
        mavros_msgs::msg::PositionTarget::IGNORE_VY |
        mavros_msgs::msg::PositionTarget::IGNORE_VZ |
        mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    msg_safe_postarget.acceleration_or_force.x = acc_sp(0);
    msg_safe_postarget.acceleration_or_force.y = acc_sp(1);
    msg_safe_postarget.acceleration_or_force.z = acc_sp(2);
    msg_safe_postarget.yaw_rate = _wz_des;
    _command_pub_postarget->publish(msg_safe_postarget);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompositeCbfNode>());
    rclcpp::shutdown();
    return 0;
}
