#include "geomagic_control/cartesianToHaptic/cartesianToHaptic_abb.hpp"
#include <kdl/frames.hpp>

namespace {
    inline void fromMsg(const geometry_msgs::msg::Wrench& in, KDL::Vector& out) {
        out.x(in.force.x);
        out.y(in.force.y);
        out.z(in.force.z);
    }
}

const extern std::string CARTESIAN_CONTROL_SERVER_NODE_NAME = "cartesian_to_haptic_abb";

// ------------------------------ Constructor ------------------------------
CartesianToHapticAbb::CartesianToHapticAbb() : CartesianToHapticBaseNode(CARTESIAN_CONTROL_SERVER_NODE_NAME) {
    setParameters();
    publishHapticCreate();
    setCartesianMode();
    subscribeHapticCreate();
    RCLCPP_INFO(this->get_logger(), "Cartesian to Haptic Feedback Node started");
}

// ------------------------------ Subscribers ------------------------------
void CartesianToHapticAbb::subscribeHapticCreate() {
    m_haptic_wrench_sub = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/jr3", 10,
        std::bind(&CartesianToHapticAbb::abbWrenchCallback, this, std::placeholders::_1));
}

void CartesianToHapticAbb::abbWrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) {

    KDL::Vector force;
    fromMsg(*msg, force);

    publishHapticFeedback(force);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianToHapticAbb>());
    rclcpp::shutdown();
    return 0;
}