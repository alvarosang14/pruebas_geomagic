// haptic_to_cartesian_node.cpp
#include "geomagic_control/hapticToCartesianNode.h"

int main(int argc, char** argv) {
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }
    
    auto node = std::make_shared<HapticToCartesianNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}