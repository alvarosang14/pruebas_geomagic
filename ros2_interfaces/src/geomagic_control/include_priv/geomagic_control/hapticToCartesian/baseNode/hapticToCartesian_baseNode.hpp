#ifndef HAPTIC_TO_CARTESIAN_BASE_NODE_H
#define HAPTIC_TO_CARTESIAN_BASE_NODE_H

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <kdl/frames.hpp>

#include "geomagic_control/hapticToCartesian/hapticToCartesian_interface.hpp"

class HapticToCartesianBaseNode : public IHapticToCartesianBaseNode {
public:
    HapticToCartesianBaseNode(const std::string & node_name);
    ~HapticToCartesianBaseNode() = default;

protected:
    void setParameters();
    void hapticSucribeCreate();
    void hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void button1Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void comprobateIntialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    geometry_msgs::msg::Pose calculateDiferentialPose(const geometry_msgs::msg::Pose::SharedPtr msg);

    double scale_x;
    double scale_y;
    double scale_z;
    bool rotation;

    KDL::Frame H_N_robot_0_sensor;
    KDL::Frame H_0_N_sensor_initial;
    KDL::Frame H_0_N_robot_initial;

    bool firstHapticOutput {false};
    bool firstRobotOutput  {false};
    bool button1_pressed {false};
    bool button2_pressed {false};

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_cmd_pub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_haptic_posePhantom_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr m_haptic_button1_sub;
};

#endif
