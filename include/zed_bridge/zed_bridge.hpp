#ifndef ZED_BRIDGE_H_
#define ZED_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#define RIGHT_IMG_FRAME_ID "zed_camera_right"
#define LEFT_IMG_FRAME_ID "zed_camera_left"
#define CAMERA_FRAME_ID "zed_camera_center"

using namespace sl;

class ZedBridge : public rclcpp::Node {

    public:
        ZedBridge();

    private:
        std::string frame_id;
        Camera zed;
        RuntimeParameters runtime_parameters;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        rclcpp::TimerBase::SharedPtr transform_timer;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;

        void publishImages();
        void broadcastTransform();

        static int getOCVtype(sl::MAT_TYPE type);
        static cv::Mat slMat2cvMat(sl::Mat& input);
};

#endif // ZED_BRIDGE_H_
