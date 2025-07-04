#include <zed_bridge/zed_bridge.hpp>


ZedBridge::ZedBridge() : Node("zed_bridge") {

    // set configuration parameters
    // https://www.stereolabs.com/docs/video/camera-controls
    InitParameters init_params;
    init_params.sdk_verbose = 1;
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.depth_minimum_distance = 0.5;
    init_params.depth_maximum_distance = 25.0;
    init_params.camera_fps = 40;
    init_params.coordinate_units = UNIT::METER;
    init_params.depth_mode = DEPTH_MODE::NEURAL_PLUS; //previous: PERFORMANCE, ULTRA, NEURAL_PLUS
    init_params.coordinate_system=COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.enable_right_side_measure = true;
    init_params.depth_stabilization = true;


    // set runtime parameters
    this->runtime_parameters.enable_depth = true;
    this->runtime_parameters.enable_fill_mode = false;
    this->runtime_parameters.confidence_threshold = 80;

    ObjectDetectionParameters obj_param;
    obj_param.enable_tracking = true;
    obj_param.enable_segmentation = false;
    obj_param.detection_model = OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS;
    obj_param.custom_onnx_file = "/home/lart-tasha/Documents/repos/ros2_ws/src/mapper_speedrun/model/yolov11n_1024_tuned.onnx";

    this->obj_runtime_param.detection_confidence_threshold = 85;
    
    // open the camera
    auto err = this->zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "FAILURE: %d %d", (int)ERROR_CODE::CAMERA_NOT_DETECTED, (int)err);
        RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera");
        rclcpp::shutdown();
    }

   // Define the ROI rectangle for AEC
    sl::Rect roi;
    roi.x = 528;
    roi.y = 720;
    roi.width = 864;
    roi.height = 372;

    // Apply ROI for AEC/AGC
    this->zed.setCameraSettings(VIDEO_SETTINGS::AEC_AGC, roi, SIDE::BOTH, true);


    PositionalTrackingParameters tracking_params;
    this->zed.enablePositionalTracking(tracking_params);
    
    auto od_ret = this->zed.enableObjectDetection(obj_param);
    if (od_ret != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "FAILURE:  %d", (int)od_ret);
        RCLCPP_ERROR(this->get_logger(), "Failed enable object detection");
        rclcpp::shutdown();
    }

    this->transform_matrix[0][0] = 1.0; this->transform_matrix[0][1] = 0.0; this->transform_matrix[0][2] = 0.0; this->transform_matrix[0][3] = -0.5;
    this->transform_matrix[1][0] = 0.0; this->transform_matrix[1][1] = 1.0; this->transform_matrix[1][2] = 0.0; this->transform_matrix[1][3] = 0.0;
    this->transform_matrix[2][0] = 0.0; this->transform_matrix[2][1] = 0.0; this->transform_matrix[2][2] = 1.0; this->transform_matrix[2][3] = 0.95;
    this->transform_matrix[3][0] = 0.0; this->transform_matrix[3][1] = 0.0; this->transform_matrix[3][2] = 0.0; this->transform_matrix[3][3] = 1.0;
    
    this->frame_counter = 0;

    this->left_image_pub  = image_transport::create_publisher(this, "/zed/left/image_raw");
    this->depth_image_pub = image_transport::create_publisher(this, "/zed/depth/image_raw");

    this->left_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/left/camera_info", 10);
    this->depth_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/depth/camera_info", 10);

    this->cone_array_pub = this->create_publisher<lart_msgs::msg::ConeArray>("/mapping/cones", 10);
    this->marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/mapping/cones_markers", 10); // changed to /mapping/markers_array previous was markers

    // initialize the transform broadcaster
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    this->transform_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ZedBridge::broadcastTransform, this));

    // start the publishing loop
    this->timer = this->create_wall_timer(std::chrono::milliseconds(1000/35), std::bind(&ZedBridge::publishImages, this));
}

void ZedBridge::publishImages() {
    //auto start_time = std::chrono::high_resolution_clock::now(); //measure function latency

    if (zed.grab(this->runtime_parameters) == ERROR_CODE::SUCCESS) {

        // retrieve the left image
        sl::Mat left_image;
        zed.retrieveImage(left_image, VIEW::LEFT);

        //Measure latency
        auto start_time = std::chrono::high_resolution_clock::now();

        zed.retrieveObjects(this->objects, this->obj_runtime_param);
        sl::Mat point_cloud;
        zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZ);
        // RCLCPP_INFO(this->get_logger(), "Number of objects detected: %ld", objects.object_list.size());
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "Object detection latency: %ld ms", duration);


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
        // sl::Mat right_image;
        // zed.retrieveImage(right_image, VIEW::RIGHT);

        // convert the image to OpenCV format
        // cv::Mat right_image_cv = slMat2cvMat(right_image);
        // // remove last channel
        // cv::cvtColor(right_image_cv, right_image_cv, cv::COLOR_BGRA2BGR);

        lart_msgs::msg::ConeArray cone_array;
        visualization_msgs::msg::MarkerArray marker_array;

        //eliminate old markers
        for (int j = 0; j < this->marker_ids.size(); j++) {
            visualization_msgs::msg::Marker old_marker;
            old_marker.header.frame_id = "base_footprint";
            old_marker.header.stamp = this->now();
            old_marker.ns = "cone_marker"; // NEW 16/05/2024 - Pedro
            old_marker.id = this->marker_ids[j];
            old_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(old_marker);
        }

        this->marker_ids.clear();  // NEW 16/05/2024 - Pedro

        for (int i = 0; i < objects.object_list.size(); i++) {
            //debug
            auto obj = objects.object_list[i];

            if (std::isnan(obj.position.x) || std::isinf(obj.position.x)) {
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Position: (%f, %f, %f)", obj.position.x, obj.position.y, obj.position.z);

            float distance = sqrt(obj.position.x * obj.position.x + obj.position.y * obj.position.y);
            if (distance < 0.5 || distance > 20.0) {
                continue;
            }
            

            float obj_x = obj.position.x;
            float obj_y = obj.position.y;
            float obj_z = obj.position.z;


            // apply the extrinsic matrix
            obj.position.x = transform_matrix[0][0] * obj_x + transform_matrix[0][1] * obj_y + transform_matrix[0][2] * obj_z + transform_matrix[0][3];
            obj.position.y = transform_matrix[1][0] * obj_x + transform_matrix[1][1] * obj_y + transform_matrix[1][2] * obj_z + transform_matrix[1][3];
            obj.position.z = transform_matrix[2][0] * obj_x + transform_matrix[2][1] * obj_y + transform_matrix[2][2] * obj_z + transform_matrix[2][3];

            //create cone message
            lart_msgs::msg::Cone cone;
            cone.position.x = obj.position.x;
            cone.position.y = obj.position.y;
            cone.position.z = 0.0;
            cone.class_type.data = obj.raw_label;

            //add cone to array
            cone_array.cones.push_back(cone);

            //create marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = this->now();
            //marker.id = (this->frame_counter + i) % 1000000000; ---> commented by Pedro
            marker.ns = "cone_marker"; // NEW 16/05/2024 - Pedro
            marker.id = this->frame_counter * 1000 + i; // NEW 16/05/2024 - Pedro
            this->marker_ids.push_back(marker.id);

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration::from_seconds(0.1); 

            marker.pose.position.x = obj.position.x;
            marker.pose.position.y = obj.position.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.23;
            marker.scale.y = 0.23;
            marker.scale.z = 0.31;

            switch(obj.raw_label) {
                case 1:
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
                    break;
                case 2:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.color.a = 1.0;
                    break;
                case 3:
                    marker.color.r = 1.0;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
                    break;
                case 4:
                    marker.color.r = 1.0;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
                    marker.scale.x = 0.35;
                    marker.scale.y = 0.35;
                    marker.scale.z = 0.50;
                    break; 
                default:
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
            }
            
            marker_array.markers.push_back(marker);
        }
        
        //publish cone array
        this->cone_array_pub->publish(cone_array);

        this->marker_array_pub->publish(marker_array);

        // convert the image to a ROS message
        // sensor_msgs::msg::Image right_image_msg;
        // right_image_msg.header.stamp = this->now();
        // right_image_msg.header.frame_id = RIGHT_IMG_FRAME_ID;
        // right_image_msg.height = right_image_cv.rows;
        // right_image_msg.width = right_image_cv.cols;
        // right_image_msg.encoding = "bgr8";
        // right_image_msg.step = right_image_cv.step;
        // right_image_msg.data = std::vector<unsigned char>(right_image_cv.data, right_image_cv.data + right_image_cv.rows * right_image_cv.cols * right_image_cv.channels());

        // retrieve the depth image
        sl::Mat depth_image;
        zed.retrieveMeasure(depth_image, MEASURE::DEPTH);


        // convert the image to a ROS message
        sensor_msgs::msg::Image depth_image_msg;
        depth_image_msg.header.stamp = this->now();
        depth_image_msg.header.frame_id = LEFT_IMG_FRAME_ID;
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

        // sensor_msgs::msg::CameraInfo right_camera_info_msg;
        // right_camera_info_msg.header.stamp = this->now();
        // right_camera_info_msg.header.frame_id = RIGHT_IMG_FRAME_ID;
        // right_camera_info_msg.height = right_image_msg.height;
        // right_camera_info_msg.width = right_image_msg.width;
        // right_camera_info_msg.distortion_model = "plumb_bob";
        // right_camera_info_msg.d.resize(5);
        // right_camera_info_msg.d[0] = calibration_params.right_cam.disto[0];
        // right_camera_info_msg.d[1] = calibration_params.right_cam.disto[1];
        // right_camera_info_msg.d[2] = calibration_params.right_cam.disto[2];
        // right_camera_info_msg.d[3] = calibration_params.right_cam.disto[3];
        // right_camera_info_msg.d[4] = calibration_params.right_cam.disto[4];
        // right_camera_info_msg.k.fill(0.0);
        // right_camera_info_msg.k[0] = calibration_params.right_cam.fx;
        // right_camera_info_msg.k[2] = calibration_params.right_cam.cx;
        // right_camera_info_msg.k[4] = calibration_params.right_cam.fy;
        // right_camera_info_msg.k[5] = calibration_params.right_cam.cy;
        // right_camera_info_msg.k[8] = 1.0;
        // right_camera_info_msg.p.fill(0.0);
        // right_camera_info_msg.p[0] = calibration_params.right_cam.fx;
        // right_camera_info_msg.p[2] = calibration_params.right_cam.cx;
        // right_camera_info_msg.p[5] = calibration_params.right_cam.fy;
        // right_camera_info_msg.p[6] = calibration_params.right_cam.cy;
        // right_camera_info_msg.p[10] = 1.0;

        sensor_msgs::msg::CameraInfo depth_camera_info_msg;
        depth_camera_info_msg.header.stamp = this->now();
        depth_camera_info_msg.header.frame_id = LEFT_IMG_FRAME_ID;
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
        //left_image_pub->publish(left_image_msg);
        //right_image_pub->publish(right_image_msg);
        //depth_image_pub->publish(depth_image_msg);
        left_image_pub.publish(left_image_msg); // PEDRO also changed this (due to the compressed images)
        //right_image_pub.publish(right_image_msg); // <- NEW 23/06/25
        depth_image_pub.publish(depth_image_msg); // <- NEW 23/06/25

        // publish the camera info
        left_info_pub->publish(left_camera_info_msg);
        //right_info_pub->publish(right_camera_info_msg);
        depth_info_pub->publish(depth_camera_info_msg);

        this->frame_counter++;
    }
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // latencies.push_back(duration.count());

    // if (latencies.size() == 500) {
    //     long long total_latency = std::accumulate(latencies.begin(), latencies.end(), 0LL);
    //     std::cout << "Average latency over 500 runs: " << total_latency / latencies.size() << " ms" << std::endl;
    //     latencies.clear();
    // }
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
