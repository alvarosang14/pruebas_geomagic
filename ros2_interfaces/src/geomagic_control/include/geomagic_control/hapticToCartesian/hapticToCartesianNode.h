#ifndef HAPTIC_TO_CARTESIAN_NODE_H
#define HAPTIC_TO_CARTESIAN_NODE_H

#include <kdl/frames.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class HapticToCartesianNode : public rclcpp::Node
{
public:
    HapticToCartesianNode();
    ~HapticToCartesianNode() = default;

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_haptic_posePhantom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_haptic_poseTeoRightArm_sub;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_cartesian_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_mode_pub;

    rclcpp::Client<SetParameters>::SharedPtr client_param_;

    KDL::Frame H_N_robot_0_sensor;
    KDL::Frame H_0_N_sensor_initial;
    KDL::Frame H_0_N_robot_initial;

    bool firstHapticOutput {false};
    bool firstTeoOutput {false};

    void publishCartesianCreate();
    bool setPoseMode();
    void setParameters();
    void hapticSucribeCreate();

    void hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void comprobateIntialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    geometry_msgs::msg::Pose calculateDiferentialPose(const geometry_msgs::msg::Pose::SharedPtr msg);

    void hapticInitialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    void teoInitialPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

};

#endif // HAPTIC_TO_CARTESIAN_NODE_H
