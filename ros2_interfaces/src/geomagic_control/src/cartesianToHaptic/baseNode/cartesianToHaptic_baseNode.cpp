#include "geomagic_control/cartesianToHaptic/baseNode/cartesianToHaptic_baseNode.hpp"

namespace {
    inline void toMsg(const KDL::Frame& in, geometry_msgs::msg::Vector3& out) {
        out.x = in.p.x();
        out.y = in.p.y();
        out.z = in.p.z();
    }

    inline double db(double v, double th) {
        return std::abs(v) <= th ? 0.0 : v;
    }
}

const extern std::string HAPTIC_YARP_DEVICE_NODE_NAME = "pepito";
const extern std::string HAPTIC_YARP_DEVICE_NODE_NAME_PARAM = "hapticDevice_nws_ros2";
constexpr double DB = 0.1;

CartesianToHapticBaseNode::CartesianToHapticBaseNode(const std::string & node_name)
: ICartesianToHapticBaseNode(node_name) {}

// ----------------------------- Parameters -----------------------------
void CartesianToHapticBaseNode::setParameters() {
    rcl_interfaces::msg::ParameterDescriptor descriptor_msg;

    double roll_N_sensor_ = 0.0;
    descriptor_msg.name = "roll_N_sensor";
    descriptor_msg.description = "Rotation about global axis X (roll) from TCP to sensor (radians).";
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    declare_parameter<double>("roll_N_sensor", 0.0, descriptor_msg);
    get_parameter("roll_N_sensor", roll_N_sensor_);

    double pitch_N_sensor_ = 0.0;
    descriptor_msg.name = "pitch_N_sensor";
    descriptor_msg.description = "Rotation about global axis Y (pitch) from TCP to sensor (radians).";
    declare_parameter<double>("pitch_N_sensor", 0.0, descriptor_msg);
    get_parameter("pitch_N_sensor", pitch_N_sensor_);

    double yaw_N_sensor_ = 0.0;
    descriptor_msg.name = "yaw_N_sensor";
    descriptor_msg.description = "Rotation about global axis Z (yaw) from TCP to sensor (radians).";
    declare_parameter<double>("yaw_N_sensor", 0.0, descriptor_msg);
    get_parameter("yaw_N_sensor", yaw_N_sensor_);

    H_sensor_haptic.M = KDL::Rotation::RPY(roll_N_sensor_, pitch_N_sensor_, yaw_N_sensor_);
}

// ------------------------------ Publisher ------------------------------
void CartesianToHapticBaseNode::publishHapticCreate() {
    m_haptic_feedback_pub = this->create_publisher<geometry_msgs::msg::Vector3>(
    HAPTIC_YARP_DEVICE_NODE_NAME + "/feedback", 10);
}

void CartesianToHapticBaseNode::publishHapticFeedback(KDL::Vector& force) {
    geometry_msgs::msg::Vector3 feedback_msg;

    force.x(db(force.x(), DB));
    force.y(db(force.y(), DB));
    force.z(db(force.z(), DB));

    force = H_sensor_haptic.M.Inverse() * force;

    RCLCPP_INFO(this->get_logger(),
                "Send to haptic wrench: [%.3f, %.3f, %.3f]",
                force.x(), force.y(), force.z());

    toMsg(KDL::Frame(force), feedback_msg);
    m_haptic_feedback_pub->publish(feedback_msg);
}

// ------------------------------ SetCartesianMode ------------------------------
bool CartesianToHapticBaseNode::setCartesianMode() {
    using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;

    RCLCPP_INFO(get_logger(), "Setting cartesian mode in external node...");
    client_param_ = create_client<SetParameters>(HAPTIC_YARP_DEVICE_NODE_NAME_PARAM + "/set_parameters");

    while (!client_param_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<SetParameters::Request>();

    Parameter param;
    param.name = "mode_feedback";
    param.value.type = rclcpp::ParameterType::PARAMETER_STRING;
    param.value.string_value = "cartesian";
    request->parameters.push_back(param);

    auto response_received_callback = [this](ServiceResponseFuture future) {
        if (future.get()->results[0].successful) {
            RCLCPP_INFO(get_logger(), "Cartesian mode correctly established in external node.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set cartesian mode.");
        }
    };

    RCLCPP_INFO(get_logger(), "Sending request to set cartesian mode...");
    auto result = client_param_->async_send_request(request, response_received_callback);
    return result.valid();
}

