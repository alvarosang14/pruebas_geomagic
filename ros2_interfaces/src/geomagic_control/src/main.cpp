#include "geomagic_control/hapticToCartesianNode.h"
#include "geomagic_control/cartesianToHapticFeedbackNode.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto haptic_to_cartesian = std::make_shared<HapticToCartesianNode>();
    auto cartesian_to_haptic = std::make_shared<CartesianToHapticFeedbackNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(haptic_to_cartesian);
    executor.add_node(cartesian_to_haptic);
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}