#ifndef HAPTIC_TO_CARTESIAN_NODE_H
#define HAPTIC_TO_CARTESIAN_NODE_H

#include <kdl/frames.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
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
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_haptic_poseTeoRightArm_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_cartesian_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_mode_pub;
    rclcpp::TimerBase::SharedPtr m_mode_timer;
    rclcpp::Client<SetParameters>::SharedPtr client_param_;

    KDL::Frame initial_haptic_pose;
    KDL::Frame initial_teo_pose;

    KDL::Frame current_haptic_pose;

    bool firstHapticOutput {false};
    bool firstTeoOutput {false};

    int m_mode_count = 0;

    void publishCartesianCreate();
    bool setPoseMode();
    void hapticSucribeCreate();

    void hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void teoPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void hapticInitialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    void teoInitialPose(const geometry_msgs::msg::Pose::SharedPtr msg);

    void calculateDiferentialPose();

};

#endif // HAPTIC_TO_CARTESIAN_NODE_H
