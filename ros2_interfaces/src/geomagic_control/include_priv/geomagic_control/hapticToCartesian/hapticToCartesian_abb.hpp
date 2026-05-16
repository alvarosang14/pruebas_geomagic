#ifndef HAPTIC_TO_CARTESIAN_ABB_HPP
#define HAPTIC_TO_CARTESIAN_ABB_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

#include "baseNode/hapticToCartesian_baseNode.hpp"

class HapticToCartesianAbb : public HapticToCartesianBaseNode {
public:
    HapticToCartesianAbb();
    ~HapticToCartesianAbb() = default;

    void publishHapticCreate() override;
    void subscribeHapticCreate() override;
    void publishDoCommand(bool do_command) override;

private:
    void abbPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_haptic_poseAbbRightArm_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_handle_cmd_pub;
};

#endif // HAPTIC_TO_CARTESIAN_ABB_HPP
