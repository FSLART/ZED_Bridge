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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <lart_msgs/msg/cone.hpp>
#include <lart_msgs/msg/cone_array.hpp>
#include <lart_msgs/msg/state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <image_transport/image_transport.hpp>
#include <cmath>
#include "rclcpp_components/register_node_macro.hpp"
#include "lart_msgs/srv/heartbeat.hpp"

// includes for latency measure
#include <chrono>
#include <numeric>

#define RIGHT_IMG_FRAME_ID "zed_camera_right"
#define LEFT_IMG_FRAME_ID "zed_camera_left"
#define CAMERA_FRAME_ID "zed_camera_center"

using namespace sl;

class ZedBridge : public rclcpp::Node
{

public:
    explicit ZedBridge(const rclcpp::NodeOptions &options);

private:
    std::string frame_id;
    Camera zed;
    RuntimeParameters runtime_parameters;
    ObjectDetectionRuntimeParameters obj_runtime_param;
    Objects objects;
    double transform_matrix[4][4];
    int64_t frame_counter;
    // marker ids array
    std::vector<int> marker_ids;
    std::chrono::steady_clock::time_point last_image_time;
    bool first_image = false;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr transform_timer;
    rclcpp::TimerBase::SharedPtr timer;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr emergency_pub;

    sl::CameraInformation cached_camera_info;
    sl::CalibrationParameters cached_calibration_params;

    // Camera info templates
    sensor_msgs::msg::CameraInfo left_camera_info_template;
    sensor_msgs::msg::CameraInfo depth_camera_info_template;

    // Add this method declaration:
    void setupCameraInfoTemplates();

    // std::shared_ptr<image_transport::ImageTransport> it; // Declare the ImageTransport object
    image_transport::Publisher left_image_pub;  // Declare the publisher for the left image
    image_transport::Publisher depth_image_pub; // Declare the publisher for the depth image

    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;

    rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr cone_array_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;
    // for latency measure
    // std::vector<long long> latencies;

    rclcpp::Time last_capture_time;
    rclcpp::Service<lart_msgs::srv::Heartbeat>::SharedPtr timestamp_service_;
    void handle_timestamp_request(const std::shared_ptr<lart_msgs::srv::Heartbeat::Request> request, std::shared_ptr<lart_msgs::srv::Heartbeat::Response> response);

    void publishImages();
    void broadcastTransform();
    void transformListener(const geometry_msgs::msg::TransformStamped &transform);

    static int getOCVtype(sl::MAT_TYPE type);
    static cv::Mat slMat2cvMat(sl::Mat &input);
};

#endif // ZED_BRIDGE_H_
