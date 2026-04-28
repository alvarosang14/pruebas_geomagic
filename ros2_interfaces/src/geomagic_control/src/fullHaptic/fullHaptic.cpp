#include "geomagic_control/fullHaptic/fullHaptic.h"

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
        in.M.GetQuaternion(out.orientation.x, out.orientation.y,
                           out.orientation.z, out.orientation.w);
    }

    inline void toMsg(const KDL::Frame& in, sensor_msgs::msg::JointState& out) {
        out.effort.resize(3);
        out.effort[0] = in.p.x();
        out.effort[1] = in.p.y();
        out.effort[2] = in.p.z();
    }
}

FullHapticNode::FullHapticNode() : Node(NODE_NAME)
{
    publishCartesianCreate();

    if (!setPoseMode()) {
        RCLCPP_ERROR(this->get_logger(), "Fallo en la configuracion pose");
        return;
    }

    if (!setFeedbackMode()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure cartesian control server");
        return;
    }

    setParameters();
    hapticSubscribeCreate();

    RCLCPP_INFO(this->get_logger(), "Full Haptic Node started");
    RCLCPP_INFO(this->get_logger(), "Listening to: /haptic_device/state/pose");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /cartesian_control_server_ros2/command/pose");
}

// ------------------------------ Publishers ------------------------------

void FullHapticNode::publishCartesianCreate()
{
    m_cartesian_cmd_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/command/pose", 10);

    m_haptic_feedback_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(HAPTIC_YARP_DEVICE_NODE_NAME) + "/feedback", 10);

    client_param_ = create_client<SetParameters>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/set_parameters");
}

// ------------------------------ Service clients ------------------------------

bool FullHapticNode::setPoseMode()
{
    using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;

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

bool FullHapticNode::setFeedbackMode()
{
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
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feedback mode set successfully");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_feedback_mode");
    }

    return result.get()->success;
}

// ------------------------------ Parameters ------------------------------

void FullHapticNode::setParameters()
{
    rcl_interfaces::msg::ParameterDescriptor descriptor_msg;
    // scale
    scale_ = 1.0;
    descriptor_msg.name = "scale";
    descriptor_msg.description = "Scaling factor to apply to the haptic movements (default: 1.0).";
    descriptor_msg.read_only = true;
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    declare_parameter<double>("scale", 1.0, descriptor_msg);
    get_parameter("scale", scale_);

    // roll
    double roll_N_sensor_ = 0.0;
    descriptor_msg.name = "roll_N_sensor";
    descriptor_msg.description = "Rotation about global axis X (roll) from TCP to sensor (radians).";
    descriptor_msg.read_only = true;
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    declare_parameter<double>("roll_N_sensor", 0.0, descriptor_msg);
    get_parameter("roll_N_sensor", roll_N_sensor_);

    // pitch
    double pitch_N_sensor_ = 0.0;
    descriptor_msg.name = "pitch_N_sensor";
    descriptor_msg.description = "Rotation about global axis Y (pitch) from TCP to sensor (radians).";
    descriptor_msg.read_only = true;
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    declare_parameter<double>("pitch_N_sensor", 0.0, descriptor_msg);
    get_parameter("pitch_N_sensor", pitch_N_sensor_);

    // yaw
    double yaw_N_sensor_ = 0.0;
    descriptor_msg.name = "yaw_N_sensor";
    descriptor_msg.description = "Rotation about global axis Z (yaw) from TCP to sensor (radians).";
    descriptor_msg.read_only = true;
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    declare_parameter<double>("yaw_N_sensor", 0.0, descriptor_msg);
    get_parameter("yaw_N_sensor", yaw_N_sensor_);

    H_N_robot_0_sensor.M = KDL::Rotation::RPY(roll_N_sensor_, pitch_N_sensor_, yaw_N_sensor_);
}

// ------------------------------ Subscribers ------------------------------

void FullHapticNode::hapticSubscribeCreate()
{
    m_haptic_posePhantom_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        std::string(HAPTIC_YARP_DEVICE_NODE_NAME) + "/state/pose", 10,
        std::bind(&FullHapticNode::hapticPoseCallback, this, std::placeholders::_1));

    m_haptic_poseTeoRightArm_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        std::string(CARTESIAN_CONTROL_SERVER_NODE_NAME) + "/state/pose", 10,
        std::bind(&FullHapticNode::teoPoseCallback, this, std::placeholders::_1));
}

// ------------------------------ Callbacks ------------------------------

void FullHapticNode::hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    hapticInitialPose(msg);

    RCLCPP_INFO(this->get_logger(),
                "Received haptic pose: [%.3f, %.3f, %.3f]",
                msg->position.x, msg->position.y, msg->position.z);

    comprobateIntialPose(msg);
}

void FullHapticNode::teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    teoInitialPose(msg);

    RCLCPP_INFO(this->get_logger(),
                "Received teo Right Arm pose: [%.3f, %.3f, %.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // --- CartesianToHapticFeedback logic ---
    fromMsg(*msg, current_teo_pose);

    KDL::Vector diff     = current_teo_pose.p - wall_center_;
    double      distance = diff.Norm();

    if (distance > wall_radius_) {
        publishHapticFeedback(KDL::Vector(0, 0, 0));
        return;
    }

    double penetration = wall_radius_ - distance;

    if (penetration > 0) {
        KDL::Vector force_direction = diff / distance;
        double force_magnitude = std::min(force_gain_ * penetration, max_force_);
        KDL::Vector force = force_direction * force_magnitude;
        publishHapticFeedback(force);
    }
}

// ------------------------------ HapticToCartesian helpers ------------------------------

void FullHapticNode::comprobateIntialPose(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (!(firstHapticOutput && firstTeoOutput)) {
        RCLCPP_INFO(this->get_logger(),
            "Not recibed intial pose of Haptic %d and Teo %d", firstHapticOutput, firstTeoOutput);
        return;
    }

    geometry_msgs::msg::Pose cartesian_cmd = calculateDiferentialPose(msg);

    m_cartesian_cmd_pub->publish(cartesian_cmd);

    RCLCPP_INFO(this->get_logger(),
                "Published cartesian command: [%.3f, %.3f, %.3f]",
                cartesian_cmd.position.x,
                cartesian_cmd.position.y,
                cartesian_cmd.position.z);
}

geometry_msgs::msg::Pose FullHapticNode::calculateDiferentialPose(
    const geometry_msgs::msg::Pose::SharedPtr msg)
{
    KDL::Frame H_0_N_sensor_current;
    fromMsg(*msg, H_0_N_sensor_current);

    KDL::Frame H_sensor_initial_current = H_0_N_sensor_initial.Inverse() * H_0_N_sensor_current;

    // Solo traslacion
    H_sensor_initial_current.M = KDL::Rotation::Identity();

    // Solo rotacion
    KDL::Frame Rot_0_N_sensor_initial = H_0_N_sensor_initial;
    Rot_0_N_sensor_initial.p = KDL::Vector::Zero();

    auto p = H_0_N_robot_initial * (H_N_robot_0_sensor * (Rot_0_N_sensor_initial * H_sensor_initial_current.p));

    p = KDL::Vector(p.x() * scale_, p.y() * scale_, p.z() * scale_);

    KDL::Frame H_0_N_robot = H_0_N_robot_initial;
    H_0_N_robot.p = p;

    geometry_msgs::msg::Pose out;
    toMsg(H_0_N_robot, out);
    return out;
}

void FullHapticNode::hapticInitialPose(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (!firstHapticOutput) {
        fromMsg(*msg, H_0_N_sensor_initial);
        firstHapticOutput = true;
        RCLCPP_INFO(get_logger(), "Initial haptic pose correctly set.");
    }
}

void FullHapticNode::teoInitialPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!firstTeoOutput) {
        fromMsg(*msg, H_0_N_robot_initial);
        firstTeoOutput = true;
        RCLCPP_INFO(get_logger(), "Initial teo pose correctly set.");
    }
}

// ------------------------------ CartesianToHapticFeedback helpers ------------------------------

void FullHapticNode::publishHapticFeedback(const KDL::Vector& force)
{
    auto feedback_msg = sensor_msgs::msg::JointState();
    toMsg(KDL::Frame(force), feedback_msg);

    m_haptic_feedback_pub->publish(feedback_msg);

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Collision! Force: [%.2f, %.2f, %.2f] N",
                         force.x(), force.y(), force.z());
}

// ------------------------------ main ------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FullHapticNode>());
    rclcpp::shutdown();
    return 0;
}
