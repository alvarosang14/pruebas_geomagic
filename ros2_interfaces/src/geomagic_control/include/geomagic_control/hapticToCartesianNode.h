#ifndef HAPTIC_TO_CARTESIAN_NODE_H
#define HAPTIC_TO_CARTESIAN_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"

class HapticToCartesianNode : public rclcpp::Node
{
public:
    HapticToCartesianNode();
    ~HapticToCartesianNode() = default;

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_haptic_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_cartesian_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_mode_pub;
    rclcpp::TimerBase::SharedPtr m_mode_timer;
    int m_mode_count = 0;

    void publishCartesianCreate();
    void setPoseMode();
    void hapticSucribeCreate();
    void hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
};

#endif // HAPTIC_TO_CARTESIAN_NODE_H
