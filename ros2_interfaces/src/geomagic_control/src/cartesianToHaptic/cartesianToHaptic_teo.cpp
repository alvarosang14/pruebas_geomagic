#include "geomagic_control/cartesianToHaptic/cartesianToHaptic_teo.hpp"

namespace {
    inline void fromMsg(const geometry_msgs::msg::PoseStamped& in, KDL::Frame& out) {
        out.p = KDL::Vector(in.pose.position.x, in.pose.position.y, in.pose.position.z);
        out.M = KDL::Rotation::Quaternion(in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w);
    }
}

const extern std::string CARTESIAN_CONTROL_SERVER_NODE_NAME = "cartesian_to_haptic_teo";

// ------------------------------ Constructor ------------------------------
CartesianToHapticTeo::CartesianToHapticTeo() : CartesianToHapticBaseNode(CARTESIAN_CONTROL_SERVER_NODE_NAME) {
    setParameters();
    setCartesianMode();
    publishHapticCreate();
    subscribeHapticCreate();
    RCLCPP_INFO(this->get_logger(), "Cartesian to Haptic Feedback Node started");
}

// ------------------------------ Subscribers ------------------------------
void CartesianToHapticTeo::subscribeHapticCreate() {
    m_haptic_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/state/pose", 10,
        std::bind(&CartesianToHapticTeo::teoPoseCallback, this, std::placeholders::_1));
}

void CartesianToHapticTeo::teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    fromMsg(*msg, current_teo_pose);
    // La norma es distancia y al dividir por la distancia (normalizar) obtenemos la direccion

    // vector ditnacia de la pose y el circulo
    KDL::Vector diff = current_teo_pose.p - wall_center_;
    // Distancia euclidia
    double distance = diff.Norm();

    if (distance > wall_radius_) {
        KDL::Vector zero(0, 0, 0);
        publishHapticFeedback(zero);
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

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianToHapticTeo>());
    rclcpp::shutdown();
    return 0;
}