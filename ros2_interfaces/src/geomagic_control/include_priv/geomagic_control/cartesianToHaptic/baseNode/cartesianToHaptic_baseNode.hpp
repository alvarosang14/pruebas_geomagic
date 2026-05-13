#ifndef CARTESIAN_TO_HAPTIC_FEEDBACK_NODE_H
#define CARTESIAN_TO_HAPTIC_FEEDBACK_NODE_H

#include <geometry_msgs/msg/vector3.hpp>
#include <kdl/frames.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include "geomagic_control/cartesianToHaptic/cartesianToHaptic_interface.hpp"

using SetParameters    = rcl_interfaces::srv::SetParameters;
using Parameter        = rcl_interfaces::msg::Parameter;
using ParameterValue   = rcl_interfaces::msg::ParameterValue;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class CartesianToHapticBaseNode : public ICartesianToHapticBaseNode {
public:
    CartesianToHapticBaseNode(const std::string & node_name);
    ~CartesianToHapticBaseNode() = default;

    void setParameters() override;
    void publishHapticCreate() override;
    void publishHapticFeedback(KDL::Vector& force) override;

protected:
    bool setCartesianMode();

private:
    rclcpp::Client<SetParameters>::SharedPtr client_param_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_haptic_feedback_pub;

    KDL::Frame H_sensor_haptic;
};

#endif