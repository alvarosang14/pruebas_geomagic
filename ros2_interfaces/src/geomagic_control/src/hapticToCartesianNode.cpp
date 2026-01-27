// haptic_to_cartesian_node.cpp
#include "geomagic_control/hapticToCartesianNode.h"

const std::string node_name = "haptic_to_cartesian_node";

HapticToCartesianNode::HapticToCartesianNode() : Node(node_name) {
    publishCartesianCreate();

    if (!setPoseMode()) {
        RCLCPP_ERROR(this->get_logger(),
                "Fallo en la configuarcion pose");
        return;
    }
    hapticSucribeCreate();

    RCLCPP_INFO(this->get_logger(),
                "Haptic to Cartesian Node started - Listening to: /haptic_device/state/pose");
    RCLCPP_INFO(this->get_logger(),
                "Publishing to: /cartesian_control_server_ros2/command/pose");
}

// ------------------------------ Publisher ------------------------------

void HapticToCartesianNode::publishCartesianCreate() {
    static std::string basePath = std::string("/") + "cartesian_control_server_ros2";

    m_cartesian_cmd_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        basePath + "/command/pose",
        10
    );
}

bool HapticToCartesianNode::setPoseMode() {
    static std::string basePath = std::string("/") + "cartesian_control_server_ros2";

    client_param_ = create_client<SetParameters>(basePath + "/set_parameters");

    // Servicio disponible????
    while (!client_param_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    // ----------------- Creacion ----------------------
    auto request = std::make_shared<SetParameters::Request>();

    Parameter param;
    param.name = "preset_streaming_cmd";
    param.value.type = rclcpp::ParameterType::PARAMETER_STRING;
    param.value.string_value = "pose";
    request->parameters.push_back(param);

    // Calling service
    using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;

    auto response_received_callback = [this](ServiceResponseFuture future)
    {
        if (future.get()->results[0].successful)
        {
            RCLCPP_INFO(get_logger(), "Preset streaming command correctly established in external node.");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to set preset streaming command.");
        }
    };

    // ---------------- resultado --------------------
    auto result = client_param_->async_send_request(request, response_received_callback);

    return result.valid();
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
