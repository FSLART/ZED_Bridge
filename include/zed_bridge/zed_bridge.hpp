#ifndef ZED_BRIDGE_H_
#define ZED_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>

class ZedBridge : public rclcpp::Node {

    public:
        ZedBridge();
        ~ZedBridge();
};

#endif // ZED_BRIDGE_H_