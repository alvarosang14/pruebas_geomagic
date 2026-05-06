#ifndef CARTESIAN_TO_HAPTIC_FEEDBACK_NODE_H
#define CARTESIAN_TO_HAPTIC_FEEDBACK_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <kdl/frames.hpp>

class CartesianToHapticFeedbackNode : public rclcpp::Node {
public:
    CartesianToHapticFeedbackNode();
    ~CartesianToHapticFeedbackNode() = default;

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_haptic_feedback_pub;

    // Subscribers
    rclcpp::SubscriptionBase::SharedPtr m_haptic_pose_sub;

    // Service client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_set_feedback_mode_;

    // State
    KDL::Frame current_teo_pose;

    // Parameters
    std::string robot_selection;

    // Parameters for virtual wall
    KDL::Vector wall_center_{0.3, 0.0, 0.4};
    double wall_radius_{0.1};
    double force_gain_{100.0};
    double max_force_{3.0};

    // Helpers
    void setParameters();
    void selectRobot();
    void publishCartesianCreate();
    void hapticSubscribeCreate();
    bool setFeedbackMode();

    // Callbacks
    void teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void abbPoseCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void abbTestPoseCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void publishHapticFeedback(const KDL::Vector& force);
};

#endif