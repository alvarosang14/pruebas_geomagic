#include "geomagic_control/cartesianToHapticFeedbackNode.h"

namespace {
    inline void fromMsg(const geometry_msgs::msg::PoseStamped& in, KDL::Frame& out) {
        out.p = KDL::Vector(in.pose.position.x, in.pose.position.y, in.pose.position.z);
        out.M = KDL::Rotation::Quaternion(in.pose.orientation.x, in.pose.orientation.y,
                                          in.pose.orientation.z, in.pose.orientation.w);
    }

    inline void toMsg(const KDL::Frame& in, sensor_msgs::msg::JointState& out) {
        out.effort.resize(3);

        out.effort[0] = in.p.x();
        out.effort[1] = in.p.y();
        out.effort[2] = in.p.z();
    }
}

CartesianToHapticFeedbackNode::CartesianToHapticFeedbackNode() : Node(NODE_NAME) {
    publishCartesianCreate();

    if (!setFeedbackMode()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure cartesian control server");
        return;
    }

    hapticSubscribeCreate();

    RCLCPP_INFO(this->get_logger(), "Cartesian to Haptic Feedback Node started");
}

// ------------------------------ Publisher ------------------------------
void CartesianToHapticFeedbackNode::publishCartesianCreate() {
    m_haptic_feedback_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(HAPTIC_YARP_DEVICE_NODE_NAME) + "/feedback", 10);

}

bool CartesianToHapticFeedbackNode::setFeedbackMode() {
    client_set_feedback_mode_ = this->create_client<yarp_control_msgs::srv::SetFeedbackMode>("set_feedback_mode");

    auto request = std::make_shared<yarp_control_msgs::srv::SetFeedbackMode::Request>();
    request->cartesian_mode = false;  // joint torque feedback mode

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
    // Suscribirse a la pose de Teo
    m_teo_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/state/pose", 10,
        std::bind(&CartesianToHapticFeedbackNode::teoPoseCallback, this, std::placeholders::_1));
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

void CartesianToHapticFeedbackNode::publishHapticFeedback(const KDL::Vector& force) {
    auto feedback_msg = sensor_msgs::msg::JointState();
    toMsg(KDL::Frame(force), feedback_msg);
    
    m_haptic_feedback_pub->publish(feedback_msg);

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "Collision! Force: [%.2f, %.2f, %.2f] N",
                            force.x(), force.y(), force.z());
}