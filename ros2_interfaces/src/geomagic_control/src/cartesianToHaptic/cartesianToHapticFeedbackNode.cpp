#include "geomagic_control/cartesianToHaptic/cartesianToHapticFeedbackNode.h"

#define NODE_NAME "cartesian_to_haptic_feedback_node"
#define CARTESIAN_CONTROL_SERVER_NODE_NAME ""
#define HAPTIC_YARP_DEVICE_NODE_NAME "pepito"

namespace {
    inline void fromMsg(const geometry_msgs::msg::PoseStamped& in, KDL::Frame& out) {
        out.p = KDL::Vector(in.pose.position.x, in.pose.position.y, in.pose.position.z);
        out.M = KDL::Rotation::Quaternion(in.pose.orientation.x, in.pose.orientation.y,
                                          in.pose.orientation.z, in.pose.orientation.w);
    }

    inline void toMsg(const KDL::Frame& in, geometry_msgs::msg::Vector3& out) {
        out.x = in.p.x();
        out.y = in.p.y();
        out.z = in.p.z();
    }
}

CartesianToHapticFeedbackNode::CartesianToHapticFeedbackNode() : Node(NODE_NAME) {
    setParameters();

    selectRobot();

    publishCartesianCreate();

    // if (!setFeedbackMode()) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to configure cartesian control server");
    //     return;
    // }

    hapticSubscribeCreate();

    RCLCPP_INFO(this->get_logger(), "Cartesian to Haptic Feedback Node started");
}

void CartesianToHapticFeedbackNode::selectRobot() {
    if (robot_selection == "teo") {
        RCLCPP_INFO(this->get_logger(), "Selected robot: Teo.");
    } else if (robot_selection == "abb") {
        RCLCPP_INFO(this->get_logger(), "Selected robot: Abb.");

    } else if (robot_selection == "abb_test") {
        RCLCPP_INFO(this->get_logger(), "Selected robot: Other Robot.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid robot selection: %s. Defaulting to 'teo'.", robot_selection.c_str());
        robot_selection = "abb";
    }
}

void CartesianToHapticFeedbackNode::setParameters() {
    rcl_interfaces::msg::ParameterDescriptor descriptor_msg;

    // robot_selection
    descriptor_msg.name = "robot_selection";
    descriptor_msg.description = "Robot to control (default: teo).";
    descriptor_msg.read_only = true;
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    declare_parameter<std::string>("robot_selection", "abb", descriptor_msg);
    get_parameter("robot_selection", robot_selection);
}

// ------------------------------ Publisher ------------------------------
void CartesianToHapticFeedbackNode::publishCartesianCreate() {
    m_haptic_feedback_pub = this->create_publisher<geometry_msgs::msg::Vector3>(
        std::string(HAPTIC_YARP_DEVICE_NODE_NAME) + "/feedback", 10);

}

bool CartesianToHapticFeedbackNode::setFeedbackMode() {
    client_set_feedback_mode_ = this->create_client<std_srvs::srv::SetBool>("set_feedback_mode");

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;  // joint torque feedback mode

    while (!client_set_feedback_mode_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client_set_feedback_mode_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feedback mode set successfully");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_feedback_mode");
    }

    return result.get()->success;
}

// ------------------------------ Subscribers ------------------------------
void CartesianToHapticFeedbackNode::hapticSubscribeCreate() {
    if (robot_selection == "teo") {
        m_haptic_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/state/pose", 10,
            std::bind(&CartesianToHapticFeedbackNode::teoPoseCallback, this, std::placeholders::_1));
    } else if (robot_selection == "abb") {
        m_haptic_pose_sub = this->create_subscription<geometry_msgs::msg::Wrench>(
            "/jr3", 10,
            std::bind(&CartesianToHapticFeedbackNode::abbPoseCallback, this, std::placeholders::_1));
    } else if (robot_selection == "abb_test") {
        m_haptic_pose_sub = this->create_subscription<geometry_msgs::msg::Wrench>(
            "/state/jr3_test", 10,
            std::bind(&CartesianToHapticFeedbackNode::abbTestPoseCallback, this, std::placeholders::_1));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported robot selection for haptic feedback: %s", robot_selection.c_str());
    }
}

void CartesianToHapticFeedbackNode::teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    fromMsg(*msg, current_teo_pose);
    // La norma es distancia y al dividir por la distancia (normalizar) obtenemos la direccion

    // vector ditnacia de la pose y el circulo
    KDL::Vector diff = current_teo_pose.p - wall_center_;
    // Distancia euclidia
    double distance = diff.Norm();

    if (distance > wall_radius_) {
        publishHapticFeedback(KDL::Vector(0, 0, 0));
        return;
    }

    double penetration = wall_radius_ - distance;

    if (penetration > 0) {
        // Normalizar: diff es direccion y distancia y queremos solo direccion
        KDL::Vector force_direction = diff / distance;
        // Como maximo la fuerza maxima que haya
        double force_magnitude = std::min(force_gain_ * penetration, max_force_);
        // Fuerza a aplicar en esa direccion
        KDL::Vector force = force_direction * force_magnitude;

        publishHapticFeedback(force);
    }
}

void CartesianToHapticFeedbackNode::abbPoseCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    KDL::Vector force(msg->force.x, msg->force.y, msg->force.z);
    publishHapticFeedback(force);
}

void CartesianToHapticFeedbackNode::abbTestPoseCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    KDL::Vector force(msg->force.x, msg->force.y, msg->force.z);
    publishHapticFeedback(force);
}

void CartesianToHapticFeedbackNode::publishHapticFeedback(const KDL::Vector& force) {
    auto feedback_msg = geometry_msgs::msg::Vector3();
    toMsg(KDL::Frame(force), feedback_msg);

    RCLCPP_INFO(this->get_logger(), "Publishing haptic feedback: [%.2f, %.2f, %.2f] N", force.x(), force.y(), force.z());

    m_haptic_feedback_pub->publish(feedback_msg);

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "Collision! Force: [%.2f, %.2f, %.2f] N",
                            force.x(), force.y(), force.z());
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianToHapticFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}