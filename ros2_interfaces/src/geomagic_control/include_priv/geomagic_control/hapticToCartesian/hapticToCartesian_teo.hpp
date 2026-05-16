#ifndef HAPTIC_TO_CARTESIAN_TEO_HPP
#define HAPTIC_TO_CARTESIAN_TEO_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include "baseNode/hapticToCartesian_baseNode.hpp"

using SetParameters    = rcl_interfaces::srv::SetParameters;
using Parameter        = rcl_interfaces::msg::Parameter;
using ParameterValue   = rcl_interfaces::msg::ParameterValue;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class HapticToCartesianTeo : public HapticToCartesianBaseNode {
public:
    HapticToCartesianTeo();
    ~HapticToCartesianTeo() = default;

    void publishHapticCreate() override;
    void subscribeHapticCreate() override;
    void publishDoCommand(bool do_command) override;

private:
    bool setPoseMode();
    void teoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Client<SetParameters>::SharedPtr client_param_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_haptic_poseTeoRightArm_sub;
};

#endif // HAPTIC_TO_CARTESIAN_TEO_HPP
