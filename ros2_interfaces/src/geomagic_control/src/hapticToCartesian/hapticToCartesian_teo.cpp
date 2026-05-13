#include "geomagic_control/hapticToCartesian/hapticToCartesian_teo.hpp"

const extern std::string CARTESIAN_CONTROL_SERVER_NODE_NAME = "haptic_to_cartesian_teo";

// ------------------------------ Constructor ------------------------------
HapticToCartesianTeo::HapticToCartesianTeo() : HapticToCartesianBaseNode(CARTESIAN_CONTROL_SERVER_NODE_NAME) {
    setParameters();
    publishHapticCreate();

    if (!setPoseMode()) {
        RCLCPP_ERROR(this->get_logger(), "Fallo en la configuracion pose");
        return;
    }

    subscribeHapticCreate();
    RCLCPP_INFO(this->get_logger(), "Haptic to Cartesian Teo Node started");
}

// ------------------------------ Publisher ------------------------------
void HapticToCartesianTeo::publishHapticCreate() {
    m_cmd_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/command/pose",
        10
    );

}

bool HapticToCartesianTeo::setPoseMode() {
    using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;

    client_param_ = create_client<SetParameters>(std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/set_parameters");

    while (!client_param_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<SetParameters::Request>();

    Parameter param;
    param.name = "preset_streaming_cmd";
    param.value.type = rclcpp::ParameterType::PARAMETER_STRING;
    param.value.string_value = "pose";
    request->parameters.push_back(param);

    auto response_received_callback = [this](ServiceResponseFuture future) {
        if (future.get()->results[0].successful) {
            RCLCPP_INFO(get_logger(), "Preset streaming command correctly established in external node.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set preset streaming command.");
        }
    };

    auto result = client_param_->async_send_request(request, response_received_callback);
    return result.valid();
}

// ------------------------------ Subscriber ------------------------------
void HapticToCartesianTeo::subscribeHapticCreate() {
    hapticSucribeCreate();
    m_haptic_poseTeoRightArm_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/state/pose",
        10,
        std::bind(&HapticToCartesianTeo::teoPoseCallback, this, std::placeholders::_1)
    );
}

void HapticToCartesianTeo::teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!firstRobotOutput) {
        H_0_N_robot_initial.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        H_0_N_robot_initial.M = KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                                           msg->pose.orientation.z, msg->pose.orientation.w);
        firstRobotOutput = true;
        RCLCPP_INFO(get_logger(), "Initial teo pose correctly set.");
    }
    RCLCPP_INFO(this->get_logger(),
                "Received teo Right Arm pose: [%.3f, %.3f, %.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

// ============================== Main ==============================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticToCartesianTeo>());
    rclcpp::shutdown();
    return 0;
}
