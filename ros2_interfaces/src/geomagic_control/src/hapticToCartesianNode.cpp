// haptic_to_cartesian_node.cpp
#include "geomagic_control/hapticToCartesianNode.h"


const std::string node_name = "haptic_to_cartesian_node";

namespace {
    inline void fromMsg(const geometry_msgs::msg::Pose& in, KDL::Frame& out) {
        out.p = KDL::Vector(in.position.x, in.position.y, in.position.z);
        out.M = KDL::Rotation::Quaternion(in.orientation.x, in.orientation.y,
                                          in.orientation.z, in.orientation.w);
    }

    inline void fromMsg(const geometry_msgs::msg::PoseStamped& in, KDL::Frame& out) {
        out.p = KDL::Vector(in.pose.position.x, in.pose.position.y, in.pose.position.z);
        out.M = KDL::Rotation::Quaternion(in.pose.orientation.x, in.pose.orientation.y,
                                          in.pose.orientation.z, in.pose.orientation.w);
    }

    inline void toMsg(const KDL::Frame& in, geometry_msgs::msg::Pose& out) {
        out.position.x = in.p.x();
        out.position.y = in.p.y();
        out.position.z = in.p.z();

        in.M.GetQuaternion(out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w);
    }
}

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
    static std::string basePathPhantom = std::string("/") + "haptic_device_ros2";

    m_haptic_posePhantom_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        basePathPhantom + "/state/pose",
        10,
        std::bind(&HapticToCartesianNode::hapticPoseCallback, this, std::placeholders::_1)
    );

    static std::string basePathTeo = std::string("/") + "cartesian_control_server_ros2";
    m_haptic_poseTeoRightArm_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        basePathTeo + "/state/pose",
        10,
        std::bind(&HapticToCartesianNode::teoPoseCallback, this, std::placeholders::_1)
    );

}

void HapticToCartesianNode::hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    hapticInitialPose(msg);

    if (firstHapticOutput && firstTeoOutput) {
        // Recibir pose del dispositivo háptico
        RCLCPP_DEBUG(this->get_logger(),
                    "Received haptic pose: [%.3f, %.3f, %.3f]",
                    msg->position.x, msg->position.y, msg->position.z);

        fromMsg(*msg, current_haptic_pose);

        calculateDiferentialPose();

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
}

void HapticToCartesianNode::calculateDiferentialPose() {
    // Al invertir se cancel base con base y me queda una matriz que relaciona el el lapiz incial con el nuevo
    KDL::Frame delta_movement = initial_haptic_pose.Inverse() * current_haptic_pose;

    // ahora aplico esa traslacion a teo
    KDL::Frame target_teo_pose = initial_teo_pose * delta_movement;

    geometry_msgs::msg::Pose cartesian_cmd;
    toMsg(target_teo_pose, cartesian_cmd);

    m_cartesian_cmd_pub->publish(cartesian_cmd);
}

void HapticToCartesianNode::teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Recibir pose del dispositivo háptico
    RCLCPP_DEBUG(this->get_logger(),
                "Received teo Right Arm pose: [%.3f, %.3f, %.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    teoInitialPose(msg);
}

void HapticToCartesianNode::hapticInitialPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!firstHapticOutput) {
        fromMsg(*msg, initial_haptic_pose);

        firstHapticOutput = true;
        RCLCPP_INFO(get_logger(), "Initial haptic pose correctly set.");
    }
}

void HapticToCartesianNode::teoInitialPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!firstTeoOutput) {
        fromMsg(*msg, initial_teo_pose);
        firstTeoOutput = true;
        RCLCPP_INFO(get_logger(), "Initial teo pose correctly set.");
    }
}
