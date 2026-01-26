// haptic_to_cartesian_node.cpp
#include "geomagic_control/hapticToCartesianNode.h"

const std::string node_name = "haptic_to_cartesian_node";

HapticToCartesianNode::HapticToCartesianNode() : Node(node_name) {
    publishCartesianCreate();
    setPoseMode();
    hapticSucribeCreate();

    RCLCPP_INFO(this->get_logger(), 
                "Haptic to Cartesian Node started - Listening to: /haptic_device/state/pose");
    RCLCPP_INFO(this->get_logger(), 
                "Publishing to: /cartesian_control_server_ros2/command/movl");
}

// ------------------------------ Publisher ------------------------------

void HapticToCartesianNode::publishCartesianCreate() {
    static std::string basePath = std::string("/") + "cartesian_control_server_ros2";

    m_cartesian_cmd_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        basePath + "/command/movl",
        10
    );

    m_mode_pub = this->create_publisher<std_msgs::msg::String>(
        basePath + "/set_mode",
        10
    );
}

void HapticToCartesianNode::setPoseMode() {
    // Timer para enviar comando de modo POSE
    m_mode_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (m_mode_count++ < 3) {
                std_msgs::msg::String mode_msg;
                mode_msg.data = "pose";
                m_mode_pub->publish(mode_msg);
                RCLCPP_INFO(this->get_logger(), "Setting controller to POSE mode");
            } else {
                m_mode_timer->cancel();
            }
        }
    );
}

// ------------------------------ Subscriber Callback ------------------------------

void HapticToCartesianNode::hapticSucribeCreate() {
    // Subscriber
    static std::string basePath = std::string("/") + "haptic_device_ros2";

    m_haptic_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        basePath + "/state/pose",
        10,
        std::bind(&HapticToCartesianNode::hapticPoseCallback, this, std::placeholders::_1)
    );

}

void HapticToCartesianNode::hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Recibir pose del dispositivo háptico
    RCLCPP_DEBUG(this->get_logger(), 
                "Received haptic pose: [%.3f, %.3f, %.3f]",
                msg->position.x, msg->position.y, msg->position.z);

    // Crear comando cartesiano con la misma pose
    geometry_msgs::msg::Pose cartesian_cmd = *msg;

    // Publicar comando al controlador cartesiano
    m_cartesian_cmd_pub->publish(cartesian_cmd);

    RCLCPP_DEBUG(this->get_logger(), 
                "Published cartesian command: [%.3f, %.3f, %.3f]",
                cartesian_cmd.position.x, 
                cartesian_cmd.position.y, 
                cartesian_cmd.position.z);
}
