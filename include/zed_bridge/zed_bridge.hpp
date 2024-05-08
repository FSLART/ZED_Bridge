#ifndef ZED_BRIDGE_H_
#define ZED_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>

using namespace sl;

class ZedBridge : public rclcpp::Node {

    public:
        ZedBridge(std::string& frame_id);
        ~ZedBridge();

    private:
        std::string frame_id;
        Camera zed;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;

        void publishImages();
};

#endif // ZED_BRIDGE_H_
