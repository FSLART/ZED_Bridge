#include <zed_bridge/zed_bridge.hpp>

ZedBridge::ZedBridge() : Node("zed_bridge") {

    // set configuration parameters
    // https://www.stereolabs.com/docs/video/camera-controls
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.camera_fps = 30;

    // open the camera
    auto err = this->zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera");
        rclcpp::shutdown();
    }

    // create the publishers
    this->left_image_pub = this->create_publisher<sensor_msgs::msg::Image>("left_image", 10);
    this->right_image_pub = this->create_publisher<sensor_msgs::msg::Image>("right_image", 10);
    this->depth_image_pub = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);

    // start the publishing loop
    this->timer = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&ZedBridge::publishImages, this));
}

void ZedBridge::publishImages() {

    if (zed.grab() == ERROR_CODE::SUCCESS) {
        // retrieve the left image
        sl::Mat left_image;
        zed.retrieveImage(left_image, VIEW::LEFT);

        // convert the image to a ROS message
        sensor_msgs::msg::Image left_image_msg;
        left_image_msg.header.stamp = this->now();
        left_image_msg.header.frame_id = FRAME_ID;
        left_image_msg.height = left_image.getHeight();
        left_image_msg.width = left_image.getWidth();
        left_image_msg.encoding = "bgr8";
        left_image_msg.step = left_image.getStepBytes();
        left_image_msg.data = std::vector<unsigned char>(left_image.getPtr<sl::uchar1>(), left_image.getPtr<sl::uchar1>() + left_image.getHeight() * left_image.getWidth() * 3);

        // retrieve the right image
        sl::Mat right_image;
        zed.retrieveImage(right_image, VIEW::RIGHT);

        // convert the image to a ROS message
        sensor_msgs::msg::Image right_image_msg;
        right_image_msg.header.stamp = this->now();
        right_image_msg.header.frame_id = FRAME_ID;
        right_image_msg.height = right_image.getHeight();
        right_image_msg.width = right_image.getWidth();
        right_image_msg.encoding = "bgr8";
        right_image_msg.step = right_image.getStepBytes();
        right_image_msg.data = std::vector<unsigned char>(right_image.getPtr<sl::uchar1>(), right_image.getPtr<sl::uchar1>() + right_image.getHeight() * right_image.getWidth() * 3);;

        // retrieve the depth image
        sl::Mat depth_image;
        zed.retrieveImage(depth_image, VIEW::DEPTH);

        // convert the image to a ROS message
        sensor_msgs::msg::Image depth_image_msg;
        depth_image_msg.header.stamp = this->now();
        depth_image_msg.header.frame_id = FRAME_ID;
        depth_image_msg.height = depth_image.getHeight();
        depth_image_msg.width = depth_image.getWidth();
        depth_image_msg.encoding = "32FC1";
        depth_image_msg.step = depth_image.getStepBytes();
        depth_image_msg.data = std::vector<unsigned char>(depth_image.getPtr<sl::uchar1>(), depth_image.getPtr<sl::uchar1>() + depth_image.getHeight() * depth_image.getWidth() * 4);

        // publish the images
        left_image_pub = this->create_publisher<sensor_msgs::msg::Image>("left_image", 10);
        left_image_pub->publish(left_image_msg);
        right_image_pub = this->create_publisher<sensor_msgs::msg::Image>("right_image", 10);
        right_image_pub->publish(right_image_msg);
        depth_image_pub = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
        depth_image_pub->publish(depth_image_msg);
    }

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedBridge>());
    rclcpp::shutdown();
    return 0;
}
