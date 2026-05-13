#ifndef CARTESIAN_TO_HAPTIC_ABB_HPP
#define CARTESIAN_TO_HAPTIC_ABB_HPP

#include <geometry_msgs/msg/wrench.hpp>

#include "baseNode/cartesianToHaptic_baseNode.hpp"

class CartesianToHapticAbb : public CartesianToHapticBaseNode {
public:
    CartesianToHapticAbb();
    ~CartesianToHapticAbb() = default;

    void subscribeHapticCreate() override;

private:
    void abbWrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr m_haptic_wrench_sub;
};

#endif // CARTESIAN_TO_HAPTIC_ABB_HPP