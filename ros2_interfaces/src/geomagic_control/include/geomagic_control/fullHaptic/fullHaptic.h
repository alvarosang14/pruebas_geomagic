#ifndef FULL_HAPTIC_NODE_H
#define FULL_HAPTIC_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <kdl/frames.hpp>

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter     = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

#define NODE_NAME                          "full_haptic_node"
#define CARTESIAN_CONTROL_SERVER_NODE_NAME "cartesian_control_server_ros2"
#define HAPTIC_YARP_DEVICE_NODE_NAME       "haptic_device_ros2"

class FullHapticNode : public rclcpp::Node
{
public:
    FullHapticNode();
    ~FullHapticNode() = default;

private:
    // ----------------------------------------------------------------
    // Publishers
    // ----------------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr       m_cartesian_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          m_mode_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   m_haptic_feedback_pub;

    // ----------------------------------------------------------------
    // Subscribers
    // ----------------------------------------------------------------
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr        m_haptic_posePhantom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_haptic_poseTeoRightArm_sub;

    // ----------------------------------------------------------------
    // Service clients
    // ----------------------------------------------------------------
    rclcpp::Client<SetParameters>::SharedPtr          client_param_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_set_feedback_mode_;

    // ----------------------------------------------------------------
    // HapticToCartesian state
    // ----------------------------------------------------------------
    KDL::Frame H_N_robot_0_sensor;
    KDL::Frame H_0_N_sensor_initial;
    KDL::Frame H_0_N_robot_initial;
    bool firstHapticOutput {false};
    bool firstTeoOutput    {false};

    // ----------------------------------------------------------------
    // CartesianToHapticFeedback state
    // ----------------------------------------------------------------
    KDL::Frame    current_teo_pose;
    KDL::Vector   wall_center_  {0.3, 0.0, 0.4};
    double        wall_radius_  {0.1};
    double        force_gain_   {100.0};
    double        max_force_    {3.0};

    // ----------------------------------------------------------------
    // Setup helpers
    // ----------------------------------------------------------------
    void publishCartesianCreate();
    bool setPoseMode();
    void setParameters();
    void hapticSubscribeCreate();
    bool setFeedbackMode();

    // ----------------------------------------------------------------
    // Callbacks
    // ----------------------------------------------------------------
    void hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void teoPoseCallback   (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // ----------------------------------------------------------------
    // HapticToCartesian helpers
    // ----------------------------------------------------------------
    void comprobateIntialPose      (const geometry_msgs::msg::Pose::SharedPtr msg);
    geometry_msgs::msg::Pose calculateDiferentialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    void hapticInitialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    void teoInitialPose   (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // ----------------------------------------------------------------
    // CartesianToHapticFeedback helpers
    // ----------------------------------------------------------------
    void publishHapticFeedback(const KDL::Vector& force);
};

#endif // FULL_HAPTIC_NODE_H