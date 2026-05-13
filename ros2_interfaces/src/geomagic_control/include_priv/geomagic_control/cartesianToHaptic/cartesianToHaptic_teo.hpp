#ifndef CARTESIAN_TO_HAPTIC_TEO_HPP
#define CARTESIAN_TO_HAPTIC_TEO_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "baseNode/cartesianToHaptic_baseNode.hpp"

class CartesianToHapticTeo : public CartesianToHapticBaseNode {
public:
    CartesianToHapticTeo();
    ~CartesianToHapticTeo() = default;
    
    void subscribeHapticCreate() override;

private:
    void teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_haptic_pose_sub;

    // State
    KDL::Frame current_teo_pose;

    // Parameters for virtual wall
    KDL::Vector wall_center_{0.3, 0.0, 0.4};
    double wall_radius_{0.1};
    double force_gain_{100.0};
    double max_force_{3.0};
};

#endif // CARTESIAN_TO_HAPTIC_TEO_HPP