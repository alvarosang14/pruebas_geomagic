#ifndef HAPTIC_TO_CARTESIAN_INTERFACE_HPP
#define HAPTIC_TO_CARTESIAN_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>

class IHapticToCartesian : public rclcpp::Node {
public:
    IHapticToCartesian(const std::string & node_name) : rclcpp::Node(node_name) {}
    virtual ~IHapticToCartesian() = default;
    virtual void subscribeHapticCreate() = 0;
};

class IHapticToCartesianBaseNode : public IHapticToCartesian {
public:
    IHapticToCartesianBaseNode(const std::string & node_name) : IHapticToCartesian(node_name) {}
    virtual ~IHapticToCartesianBaseNode() = default;
    virtual void publishHapticCreate() = 0;
};

#endif