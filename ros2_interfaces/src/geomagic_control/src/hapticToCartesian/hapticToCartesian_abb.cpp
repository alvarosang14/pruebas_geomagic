#include "geomagic_control/hapticToCartesian/hapticToCartesian_abb.hpp"

const extern std::string CARTESIAN_CONTROL_SERVER_NODE_NAME = "haptic_to_cartesian_abb";

// ------------------------------ Constructor ------------------------------
HapticToCartesianAbb::HapticToCartesianAbb() : HapticToCartesianBaseNode(CARTESIAN_CONTROL_SERVER_NODE_NAME) {
    setParameters();
    publishHapticCreate();
    subscribeHapticCreate();
    RCLCPP_INFO(this->get_logger(), "Haptic to Cartesian Abb Node started");
}

// ------------------------------ Publisher ------------------------------
void HapticToCartesianAbb::publishHapticCreate() {
    m_cmd_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        "/command/pose",
        10
    );
}

// ------------------------------ Subscriber ------------------------------
void HapticToCartesianAbb::subscribeHapticCreate() {
    hapticSucribeCreate();
    m_haptic_poseAbbRightArm_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        "/state/pose",
        10,
        std::bind(&HapticToCartesianAbb::abbPoseCallback, this, std::placeholders::_1)
    );
}

void HapticToCartesianAbb::abbPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!firstRobotOutput) {
        KDL::Frame frame;
        frame.p = KDL::Vector(msg->position.x, msg->position.y, msg->position.z);
        frame.M = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y,
                                             msg->orientation.z, msg->orientation.w);
        H_0_N_robot_initial = frame;
        firstRobotOutput = true;
        RCLCPP_INFO(get_logger(), "Initial abb pose correctly set.");
    }
}

// ============================== Main ==============================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticToCartesianAbb>());
    rclcpp::shutdown();
    return 0;
}
