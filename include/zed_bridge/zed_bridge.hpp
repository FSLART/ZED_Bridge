#ifndef ZED_BRIDGE_H_
#define ZED_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

#define FRAME_ID "zed_camera_center"

using namespace sl;

class ZedBridge : public rclcpp::Node {

    public:
        ZedBridge();

    private:
        std::string frame_id;
        Camera zed;
        RuntimeParameters runtime_parameters;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;

        void publishImages();

        static int getOCVtype(sl::MAT_TYPE type);
        static cv::Mat slMat2cvMat(sl::Mat& input);
};

#endif // ZED_BRIDGE_H_
