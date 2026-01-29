// haptic_to_cartesian_node.cpp
#include "geomagic_control/hapticToCartesianNode.h"

#define NODE_NAME "haptic_to_cartesian_node"
#define CARTESIAN_CONTROL_SERVER_NODE_NAME "cartesian_control_server_ros2"
#define HAPTIC_YARP_DEVICE_NODE_NAME "haptic_device_ros2"

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

HapticToCartesianNode::HapticToCartesianNode() : Node(NODE_NAME) {
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
    m_cartesian_cmd_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/command/pose",
        10
    );

    client_param_ = create_client<SetParameters>(std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/set_parameters");
}

bool HapticToCartesianNode::setPoseMode() {
    using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;

    // wait for service
    while (!client_param_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    // param
    auto request = std::make_shared<SetParameters::Request>();

    Parameter param;
    param.name = "preset_streaming_cmd";
    param.value.type = rclcpp::ParameterType::PARAMETER_STRING;
    param.value.string_value = "pose";
    request->parameters.push_back(param);

    // calling
    auto response_received_callback = [this](ServiceResponseFuture future) {
        if (future.get()->results[0].successful) {
            RCLCPP_INFO(get_logger(), "Preset streaming command correctly established in external node.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set preset streaming command.");
        }
    };

    // result future
    auto result = client_param_->async_send_request(request, response_received_callback);

    return result.valid();
}

// ------------------------------ Subscriber Callback ------------------------------
void HapticToCartesianNode::hapticSucribeCreate() {
    // Subscriber
    m_haptic_posePhantom_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        std::string(HAPTIC_YARP_DEVICE_NODE_NAME) + "/state/pose",
        10,
        std::bind(&HapticToCartesianNode::hapticPoseCallback, this, std::placeholders::_1)
    );

    m_haptic_poseTeoRightArm_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/state/pose",
        10,
        std::bind(&HapticToCartesianNode::teoPoseCallback, this, std::placeholders::_1)
    );

}

void HapticToCartesianNode::hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    hapticInitialPose(msg);

    // Recibir pose del dispositivo háptico
    RCLCPP_DEBUG(this->get_logger(),
                "Received haptic pose: [%.3f, %.3f, %.3f]",
                msg->position.x, msg->position.y, msg->position.z);
    
    comprobateIntialPose(msg);
}

void HapticToCartesianNode::teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Recibir pose del dispositivo háptico
    teoInitialPose(msg);

    RCLCPP_DEBUG(this->get_logger(),
                "Received teo Right Arm pose: [%.3f, %.3f, %.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

}

// ------------------------------ Helpers ------------------------------
void HapticToCartesianNode::comprobateIntialPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!(firstHapticOutput && firstTeoOutput)) {
        RCLCPP_INFO(this->get_logger(),
            "Not recibed intial pose of Haptic %d and Teo %d", firstHapticOutput, firstTeoOutput);
        return;
    }

    geometry_msgs::msg::Pose cartesian_cmd = calculateDiferentialPose(msg);

    m_cartesian_cmd_pub->publish(cartesian_cmd);

    RCLCPP_DEBUG(this->get_logger(),
                "Published cartesian command: [%.3f, %.3f, %.3f]",
                cartesian_cmd.position.x,
                cartesian_cmd.position.y,
                cartesian_cmd.position.z);
}

geometry_msgs::msg::Pose HapticToCartesianNode::calculateDiferentialPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
    KDL::Frame current_haptic_pose;
    fromMsg(*msg, current_haptic_pose);

    // Al invertir se cancel base con base y me queda una matriz que relaciona el el lapiz incial con el nuevo
    KDL::Frame delta_movement = initial_haptic_pose.Inverse() * current_haptic_pose;

    // ahora aplico esa traslacion a teo
    KDL::Frame target_teo_pose = initial_teo_pose * delta_movement;

    geometry_msgs::msg::Pose cartesian_cmd;
    toMsg(target_teo_pose, cartesian_cmd);

    return cartesian_cmd;
}

// ------------------------------ Initial Pose Setters ------------------------------
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
