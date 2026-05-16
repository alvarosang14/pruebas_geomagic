#include "geomagic_control/hapticToCartesian/baseNode/hapticToCartesian_baseNode.hpp"

namespace {
    inline void fromMsg(const geometry_msgs::msg::Pose& in, KDL::Frame& out) {
        out.p = KDL::Vector(in.position.x, in.position.y, in.position.z);
        out.M = KDL::Rotation::Quaternion(in.orientation.x, in.orientation.y,
                                          in.orientation.z, in.orientation.w);
    }

    inline void toMsg(const KDL::Frame& in, geometry_msgs::msg::Pose& out) {
        out.position.x = in.p.x();
        out.position.y = in.p.y();
        out.position.z = in.p.z();

        in.M.GetQuaternion(out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w);
    }
}

const extern std::string HAPTIC_YARP_DEVICE_NODE_NAME = "pepito";

// ----------------------------- Constructor -----------------------------
HapticToCartesianBaseNode::HapticToCartesianBaseNode(const std::string & node_name)
: IHapticToCartesianBaseNode(node_name) {}

// ----------------------------- Parameters -----------------------------
void HapticToCartesianBaseNode::setParameters() {
    rcl_interfaces::msg::ParameterDescriptor descriptor_msg;

    scale_x = 1.0;
    descriptor_msg.name = "scale_x";
    descriptor_msg.description = "Scaling factor to apply to the haptic movements (default: 1.0).";
    descriptor_msg.read_only = true;
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    declare_parameter<double>("scale_x", 1.0, descriptor_msg);
    get_parameter("scale_x", scale_x);

    scale_y = 1.0;
    descriptor_msg.name = "scale_y";
    descriptor_msg.description = "Scaling factor to apply to the haptic movements (default: 1.0).";
    declare_parameter<double>("scale_y", 1.0, descriptor_msg);
    get_parameter("scale_y", scale_y);

    scale_z = 1.0;
    descriptor_msg.name = "scale_z";
    descriptor_msg.description = "Scaling factor to apply to the haptic movements (default: 1.0).";
    declare_parameter<double>("scale_z", 1.0, descriptor_msg);
    get_parameter("scale_z", scale_z);

    rotation = true;
    descriptor_msg.name = "rotation";
    descriptor_msg.description = "Whether to consider the rotation of the haptic device (default: true).";
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
    declare_parameter<bool>("rotation", true, descriptor_msg);
    get_parameter("rotation", rotation);

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

    H_N_robot_0_sensor.M = KDL::Rotation::RPY(roll_N_sensor_, pitch_N_sensor_, yaw_N_sensor_);
}

// ------------------------------ Subscriber Callback ------------------------------
void HapticToCartesianBaseNode::hapticSucribeCreate() {
    m_haptic_posePhantom_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        HAPTIC_YARP_DEVICE_NODE_NAME + "/state/pose",
        10,
        std::bind(&HapticToCartesianBaseNode::hapticPoseCallback, this, std::placeholders::_1)
    );

    m_haptic_button1_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        HAPTIC_YARP_DEVICE_NODE_NAME + "/state/button1",
        10,
        std::bind(&HapticToCartesianBaseNode::button1Callback, this, std::placeholders::_1)
    );
}

void HapticToCartesianBaseNode::button1Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!msg->data.empty() && msg->data[0] == 1) {
        RCLCPP_INFO(this->get_logger(), "Button 1 pressed. Resetting initial poses.");

        button1_pressed = !button1_pressed;

        firstHapticOutput = false;
        firstRobotOutput = false;

        H_0_N_sensor_initial = KDL::Frame::Identity();
        H_0_N_robot_initial = KDL::Frame::Identity();
    }

    if (msg->data.size() > 1 && msg->data[1] == 1) {
        RCLCPP_INFO(this->get_logger(), "Button 2 pressed. Resetting initial robot pose.");

        publishDoCommand(true);
    }
}

void HapticToCartesianBaseNode::hapticPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!firstRobotOutput) {
        RCLCPP_INFO(this->get_logger(), "Waiting for initial robot pose...");
        return;
    }

    if (!firstHapticOutput) {
        fromMsg(*msg, H_0_N_sensor_initial);
        firstHapticOutput = true;
        RCLCPP_INFO(get_logger(), "Initial haptic pose correctly set.");
    }

    RCLCPP_INFO(this->get_logger(),
                "Received haptic pose: [%.3f, %.3f, %.3f]",
                msg->position.x, msg->position.y, msg->position.z);

    comprobateIntialPose(msg);
}

// ------------------------------ Helpers ------------------------------
void HapticToCartesianBaseNode::comprobateIntialPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!(firstHapticOutput && firstRobotOutput)) {
        RCLCPP_INFO(this->get_logger(),
            "Not received initial pose of Haptic %d and Robot %d", firstHapticOutput, firstRobotOutput);
        return;
    }

    geometry_msgs::msg::Pose cartesian_cmd = calculateDiferentialPose(msg);
    m_cmd_pub->publish(cartesian_cmd);

    RCLCPP_INFO(this->get_logger(),
                "Published cartesian command: [%.3f, %.3f, %.3f]",
                cartesian_cmd.position.x,
                cartesian_cmd.position.y,
                cartesian_cmd.position.z);
}

geometry_msgs::msg::Pose HapticToCartesianBaseNode::calculateDiferentialPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
    KDL::Frame H_0_N_sensor_current;

    // RCLCPP_INFO(this->get_logger(), "Msg: [%.3f, %.3f, %.3f]", msg->position.x, msg->position.y, msg->position.z);
    fromMsg(*msg, H_0_N_sensor_current);

    KDL::Frame H_sensor_initial_current = H_0_N_sensor_initial.Inverse() * H_0_N_sensor_current;

    //solo traslación
    H_sensor_initial_current.M = KDL::Rotation::Identity();

    // Solo rotacion
    KDL::Frame Rot_0_N_sensor_initial = H_0_N_sensor_initial;
    Rot_0_N_sensor_initial.p = KDL::Vector::Zero();
    // H_N_robot_0_sensor ya nos hemos asegurado que es solo rotacion

    auto p = H_0_N_robot_initial * (H_N_robot_0_sensor * (Rot_0_N_sensor_initial * H_sensor_initial_current.p));

    // RCLCPP_INFO(this->get_logger(), "Calculated position before scaling: [%.3f, %.3f, %.3f]", p.x(), p.y(), p.z());
    
    //escale??
    p = KDL::Vector(p.x() * scale_x, p.y() * scale_y, p.z() * scale_z);

    RCLCPP_INFO(this->get_logger(), "Calculated position after scaling: [%.3f, %.3f, %.3f]", p.x(), p.y(), p.z());

    // transformamos la traslacion con respecto la base del sensor
    // tranformamos el resultado a los mismos ejes de teo
    // transformamos el resultado con la base de teo
    // KDL::Frame H_0_N_robot = H_0_N_robot_initial * H_N_robot_0_sensor * Rot_0_N_sensor_initial * H_sensor_initial_current;
    KDL::Frame H_0_N_robot = H_0_N_robot_initial;
    H_0_N_robot.p = p;

    geometry_msgs::msg::Pose out;
    toMsg(H_0_N_robot, out);

    // RCLCPP_INFO(this->get_logger(),
    //             "Calculated cartesian command: [%.3f, %.3f, %.3f]",
    //             out.position.x, out.position.y, out.position.z);
    return out;
}