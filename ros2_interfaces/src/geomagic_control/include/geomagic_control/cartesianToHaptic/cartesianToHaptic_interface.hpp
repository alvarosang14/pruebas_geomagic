#ifndef CARTESIAN_TO_HAPTIC_INTERFACE_HPP
#define CARTESIAN_TO_HAPTIC_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>

class ICartesianToHaptic : public rclcpp::Node {
public:
    ICartesianToHaptic(const std::string & node_name) : rclcpp::Node(node_name) {}
    virtual ~ICartesianToHaptic() = default;
    virtual void subscribeHapticCreate() = 0;
};

class ICartesianToHapticBaseNode : public ICartesianToHaptic {
public:
    ICartesianToHapticBaseNode(const std::string & node_name) : ICartesianToHaptic(node_name) {}
    virtual ~ICartesianToHapticBaseNode() = default;
    virtual void setParameters() = 0;
    virtual void publishHapticCreate() = 0;
    virtual void publishHapticFeedback(KDL::Vector & force) = 0;
};

#endif