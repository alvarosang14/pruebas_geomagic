#ifndef CARTESIAN_TO_HAPTIC_FEEDBACK_NODE_H
#define CARTESIAN_TO_HAPTIC_FEEDBACK_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <kdl/frames.hpp>

#define NODE_NAME "cartesian_to_haptic_feedback_node"
#define CARTESIAN_CONTROL_SERVER_NODE_NAME "cartesian_control_server_ros2"
#define HAPTIC_YARP_DEVICE_NODE_NAME "haptic_device_ros2"

class CartesianToHapticFeedbackNode : public rclcpp::Node {
public:
    CartesianToHapticFeedbackNode();
    ~CartesianToHapticFeedbackNode() = default;

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_haptic_feedback_pub;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_teo_pose_sub;

    // Service client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_set_feedback_mode_;

    // State
    KDL::Frame current_teo_pose;

    // Parameters for virtual wall
    KDL::Vector wall_center_{0.3, 0.0, 0.4};
    double wall_radius_{0.1};
    double force_gain_{100.0};
    double max_force_{3.0};

    // Helpers
    void publishCartesianCreate();
    void hapticSubscribeCreate();
    bool setFeedbackMode();

    // Callbacks
    void teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishHapticFeedback(const KDL::Vector& force);
};

#endif