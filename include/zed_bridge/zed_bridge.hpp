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
#include <lart_msgs/msg/cone.hpp>
#include <lart_msgs/msg/cone_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <image_transport/image_transport.hpp>
#include <cmath>


//includes for latency measure
#include <chrono>
#include <numeric>

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
        ObjectDetectionRuntimeParameters obj_runtime_param;
        Objects objects;
        double transform_matrix[4][4];
        int64_t frame_counter;
        //marker ids array
        std::vector<int> marker_ids;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        rclcpp::TimerBase::SharedPtr transform_timer;
        rclcpp::TimerBase::SharedPtr timer;


        // std::shared_ptr<image_transport::ImageTransport> it; // Declare the ImageTransport object
        image_transport::Publisher left_image_pub;           // Declare the publisher for the left image
        image_transport::Publisher depth_image_pub;          // Declare the publisher for the depth image

        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;

        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub;
        // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
        rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr cone_array_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;
        //for latency measure
        //std::vector<long long> latencies;

        void publishImages();
        void broadcastTransform();

        static int getOCVtype(sl::MAT_TYPE type);
        static cv::Mat slMat2cvMat(sl::Mat& input);
};

#endif // ZED_BRIDGE_H_
