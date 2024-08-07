#include <zed_bridge/zed_bridge.hpp>

ZedBridge::ZedBridge() : Node("zed_bridge") {

    // set configuration parameters
    // https://www.stereolabs.com/docs/video/camera-controls
    InitParameters init_params;
    init_params.sdk_verbose = 1;
    init_params.camera_resolution = RESOLUTION::HD720;
    init_params.depth_minimum_distance = 1.0;
    init_params.depth_maximum_distance = 35.0;
    init_params.camera_fps = 60;
    init_params.coordinate_units = UNIT::METER;
    init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_params.enable_right_side_measure = true;

    // set runtime parameters
    this->runtime_parameters.enable_depth = true;
    this->runtime_parameters.enable_fill_mode = true;

    // open the camera
    auto err = this->zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "FAILURE: %d %d", (int)ERROR_CODE::CAMERA_NOT_DETECTED, (int)err);
        RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera");
        rclcpp::shutdown();
    }

    // create the publishers
    this->left_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/zed/left/image_raw", 10);
    this->right_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/zed/image_raw", 10);
    this->depth_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/zed/depth/image_raw", 10);
    this->left_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/left/camera_info", 10);
    this->right_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/right/camera_info", 10);
    this->depth_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/depth/camera_info", 10);

    // initialize the transform broadcaster
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    this->transform_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ZedBridge::broadcastTransform, this));

    // start the publishing loop
    this->timer = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&ZedBridge::publishImages, this));
}

void ZedBridge::publishImages() {

    if (zed.grab(this->runtime_parameters) == ERROR_CODE::SUCCESS) {
        // retrieve the left image
        sl::Mat left_image;
        zed.retrieveImage(left_image, VIEW::LEFT);

        // convert the image to OpenCV format
        cv::Mat left_image_cv = slMat2cvMat(left_image);
        // remove last channel
        cv::cvtColor(left_image_cv, left_image_cv, cv::COLOR_BGRA2BGR);

        // convert the image to a ROS message
        sensor_msgs::msg::Image left_image_msg;
        left_image_msg.header.stamp = this->now();
        left_image_msg.header.frame_id = LEFT_IMG_FRAME_ID;
        left_image_msg.height = left_image_cv.rows;
        left_image_msg.width = left_image_cv.cols;
        left_image_msg.encoding = "bgr8";
        left_image_msg.step = left_image_cv.step;
        left_image_msg.data = std::vector<unsigned char>(left_image_cv.data, left_image_cv.data + left_image_cv.rows * left_image_cv.cols * left_image_cv.channels());

        // retrieve the right image
        sl::Mat right_image;
        zed.retrieveImage(right_image, VIEW::RIGHT);

        // convert the image to OpenCV format
        cv::Mat right_image_cv = slMat2cvMat(right_image);
        // remove last channel
        cv::cvtColor(right_image_cv, right_image_cv, cv::COLOR_BGRA2BGR);

        // convert the image to a ROS message
        sensor_msgs::msg::Image right_image_msg;
        right_image_msg.header.stamp = this->now();
        right_image_msg.header.frame_id = RIGHT_IMG_FRAME_ID;
        right_image_msg.height = right_image_cv.rows;
        right_image_msg.width = right_image_cv.cols;
        right_image_msg.encoding = "bgr8";
        right_image_msg.step = right_image_cv.step;
        right_image_msg.data = std::vector<unsigned char>(right_image_cv.data, right_image_cv.data + right_image_cv.rows * right_image_cv.cols * right_image_cv.channels());

        // retrieve the depth image
        sl::Mat depth_image;
        zed.retrieveMeasure(depth_image, MEASURE::DEPTH_RIGHT);

        // convert the image to a ROS message
        sensor_msgs::msg::Image depth_image_msg;
        depth_image_msg.header.stamp = this->now();
        depth_image_msg.header.frame_id = RIGHT_IMG_FRAME_ID;
        depth_image_msg.height = depth_image.getHeight();
        depth_image_msg.width = depth_image.getWidth();
        depth_image_msg.encoding = "32FC1";
        depth_image_msg.step = depth_image.getStepBytes();
        depth_image_msg.data = std::vector<unsigned char>(depth_image.getPtr<sl::uchar1>(), depth_image.getPtr<sl::uchar1>() + depth_image.getHeight() * depth_image.getWidth() * 4);

        // get the camera calibration parameters
        sl::CameraInformation camera_info = zed.getCameraInformation();
        sl::CalibrationParameters calibration_params = camera_info.camera_configuration.calibration_parameters;

        // create the camera info messages
        sensor_msgs::msg::CameraInfo left_camera_info_msg;
        left_camera_info_msg.header.stamp = this->now();
        left_camera_info_msg.header.frame_id = LEFT_IMG_FRAME_ID;
        left_camera_info_msg.height = left_image_msg.height;
        left_camera_info_msg.width = left_image_msg.width;
        left_camera_info_msg.distortion_model = "plumb_bob";
        left_camera_info_msg.d.resize(5);
        left_camera_info_msg.d[0] = calibration_params.left_cam.disto[0];
        left_camera_info_msg.d[1] = calibration_params.left_cam.disto[1];
        left_camera_info_msg.d[2] = calibration_params.left_cam.disto[2];
        left_camera_info_msg.d[3] = calibration_params.left_cam.disto[3];
        left_camera_info_msg.d[4] = calibration_params.left_cam.disto[4];
        left_camera_info_msg.k.fill(0.0);
        left_camera_info_msg.k[0] = calibration_params.left_cam.fx;
        left_camera_info_msg.k[2] = calibration_params.left_cam.cx;
        left_camera_info_msg.k[4] = calibration_params.left_cam.fy;
        left_camera_info_msg.k[5] = calibration_params.left_cam.cy;
        left_camera_info_msg.k[8] = 1.0;
        left_camera_info_msg.p.fill(0.0);
        left_camera_info_msg.p[0] = calibration_params.left_cam.fx;
        left_camera_info_msg.p[2] = calibration_params.left_cam.cx;
        left_camera_info_msg.p[5] = calibration_params.left_cam.fy;
        left_camera_info_msg.p[6] = calibration_params.left_cam.cy;
        left_camera_info_msg.p[10] = 1.0;

        sensor_msgs::msg::CameraInfo right_camera_info_msg;
        right_camera_info_msg.header.stamp = this->now();
        right_camera_info_msg.header.frame_id = RIGHT_IMG_FRAME_ID;
        right_camera_info_msg.height = right_image_msg.height;
        right_camera_info_msg.width = right_image_msg.width;
        right_camera_info_msg.distortion_model = "plumb_bob";
        right_camera_info_msg.d.resize(5);
        right_camera_info_msg.d[0] = calibration_params.right_cam.disto[0];
        right_camera_info_msg.d[1] = calibration_params.right_cam.disto[1];
        right_camera_info_msg.d[2] = calibration_params.right_cam.disto[2];
        right_camera_info_msg.d[3] = calibration_params.right_cam.disto[3];
        right_camera_info_msg.d[4] = calibration_params.right_cam.disto[4];
        right_camera_info_msg.k.fill(0.0);
        right_camera_info_msg.k[0] = calibration_params.right_cam.fx;
        right_camera_info_msg.k[2] = calibration_params.right_cam.cx;
        right_camera_info_msg.k[4] = calibration_params.right_cam.fy;
        right_camera_info_msg.k[5] = calibration_params.right_cam.cy;
        right_camera_info_msg.k[8] = 1.0;
        right_camera_info_msg.p.fill(0.0);
        right_camera_info_msg.p[0] = calibration_params.right_cam.fx;
        right_camera_info_msg.p[2] = calibration_params.right_cam.cx;
        right_camera_info_msg.p[5] = calibration_params.right_cam.fy;
        right_camera_info_msg.p[6] = calibration_params.right_cam.cy;
        right_camera_info_msg.p[10] = 1.0;

        sensor_msgs::msg::CameraInfo depth_camera_info_msg;
        depth_camera_info_msg.header.stamp = this->now();
        depth_camera_info_msg.header.frame_id = RIGHT_IMG_FRAME_ID;
        depth_camera_info_msg.height = depth_image_msg.height;
        depth_camera_info_msg.width = depth_image_msg.width;
        depth_camera_info_msg.distortion_model = "plumb_bob";
        depth_camera_info_msg.d.resize(5);
        depth_camera_info_msg.d[0] = calibration_params.left_cam.disto[0];
        depth_camera_info_msg.d[1] = calibration_params.left_cam.disto[1];
        depth_camera_info_msg.d[2] = calibration_params.left_cam.disto[2];
        depth_camera_info_msg.d[3] = calibration_params.left_cam.disto[3];
        depth_camera_info_msg.d[4] = calibration_params.left_cam.disto[4];
        depth_camera_info_msg.k.fill(0.0);
        depth_camera_info_msg.k[0] = calibration_params.left_cam.fx;
        depth_camera_info_msg.k[2] = calibration_params.left_cam.cx;
        depth_camera_info_msg.k[4] = calibration_params.left_cam.fy;
        depth_camera_info_msg.k[5] = calibration_params.left_cam.cy;
        depth_camera_info_msg.k[8] = 1.0;
        depth_camera_info_msg.p.fill(0.0);
        depth_camera_info_msg.p[0] = calibration_params.left_cam.fx;
        depth_camera_info_msg.p[2] = calibration_params.left_cam.cx;
        depth_camera_info_msg.p[5] = calibration_params.left_cam.fy;
        depth_camera_info_msg.p[6] = calibration_params.left_cam.cy;
        depth_camera_info_msg.p[10] = 1.0;

        // publish the images
        left_image_pub->publish(left_image_msg);
        right_image_pub->publish(right_image_msg);
        depth_image_pub->publish(depth_image_msg);

        // publish the camera info
        left_info_pub->publish(left_camera_info_msg);
        right_info_pub->publish(right_camera_info_msg);
        depth_info_pub->publish(depth_camera_info_msg);
    }

}

void ZedBridge::broadcastTransform() {

    // get the camera calibration parameters
    sl::CameraInformation camera_info = zed.getCameraInformation();
    sl::CalibrationParameters calibration_params = camera_info.camera_configuration.calibration_parameters;
    float baseline = calibration_params.getCameraBaseline();

    // create the right lens transform
    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion q;
    

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = CAMERA_FRAME_ID;
    transform.child_frame_id = RIGHT_IMG_FRAME_ID;
    transform.transform.translation.x = 0;
    transform.transform.translation.y = -baseline/2.0;
    transform.transform.translation.z = 0;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    // broadcast the transform
    this->tf_broadcaster->sendTransform(transform);

    // create the left lens transform
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = CAMERA_FRAME_ID;
    transform.child_frame_id = LEFT_IMG_FRAME_ID;
    transform.transform.translation.x = 0;
    transform.transform.translation.y = baseline/2.0;
    transform.transform.translation.z = 0;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // broadcast the transform
    this->tf_broadcaster->sendTransform(transform);
}

// Mapping between MAT_TYPE and CV_TYPE
int ZedBridge::getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat ZedBridge::slMat2cvMat(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedBridge>());
    rclcpp::shutdown();
    return 0;
}
