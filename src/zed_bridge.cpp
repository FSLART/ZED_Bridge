#include <zed_bridge/zed_bridge.hpp>


ZedBridge::ZedBridge() : Node("zed_bridge") {

    this->emergency_pub = this->create_publisher<lart_msgs::msg::State>("/pc_origin/emergency", 10);
    
    // set configuration parameters
    // https://www.stereolabs.com/docs/video/camera-controls
    InitParameters init_params;
    init_params.sdk_verbose = 1;
    init_params.camera_resolution = RESOLUTION::HD1200;
    init_params.depth_minimum_distance = 0.5;
    init_params.depth_maximum_distance = 25.0;
    init_params.camera_fps = 30;
    init_params.coordinate_units = UNIT::METER;
    init_params.depth_mode = DEPTH_MODE::NEURAL_PLUS; //previous: PERFORMANCE, ULTRA, NEURAL_PLUS
    init_params.coordinate_system=COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.enable_right_side_measure = true;
    init_params.depth_stabilization = true;
    // init_params.input.setFromStream("192.168.144.124", 30000);


    // set runtime parameters
    this->runtime_parameters.enable_depth = true;
    this->runtime_parameters.enable_fill_mode = false;
    this->runtime_parameters.confidence_threshold = 70;

    ObjectDetectionParameters obj_param;
    obj_param.enable_tracking = false;
    obj_param.enable_segmentation = false;
    obj_param.detection_model = OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS;
    obj_param.custom_onnx_file = "/home/lart-tasha/Documents/repos/ros2_ws/src/mapper_speedrun/model/yolo_v8_n.onnx";
    // obj_param.custom_onnx_file = "/home/andre-lopes/Desktop/ros2_ws/src/mapper_speedrun/model/yolov11n_1024_tuned.onnx";



    this->obj_runtime_param.detection_confidence_threshold = 85;
    
    // open the camera
    auto err = this->zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "FAILURE: %d %d", (int)ERROR_CODE::CAMERA_NOT_DETECTED, (int)err);
        RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera");
        lart_msgs::msg::State emergency;
        emergency.data= lart_msgs::msg::State::EMERGENCY;
        this->emergency_pub->publish(emergency);
        rclcpp::shutdown();
    }

   // Define the ROI rectangle for AEC
    sl::Rect roi;
    roi.x = 960;
    roi.y = 600;
    roi.width = 1030;
    roi.height = 400;

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

    // this->left_image_pub = this->create_publisher<image_transport::Image>("/zed/left/image_raw", 10);
    // this->depth_image_pub = this->create_publisher<image_transport::Image>("/zed/depth/image_raw", 10);

    this->left_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/left/camera_info", 10);
    this->depth_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/zed/depth/camera_info", 10);

    this->cone_array_pub = this->create_publisher<lart_msgs::msg::ConeArray>("/mapping/cones", 10);
    this->marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/mapping/cones_markers", 10); // changed to /mapping/markers_array previous was markers

    // initialize the transform broadcaster
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // this->transform_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ZedBridge::broadcastTransform, this));

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Cache camera info once instead of getting it every frame
    this->cached_camera_info = zed.getCameraInformation();
    this->cached_calibration_params = cached_camera_info.camera_configuration.calibration_parameters;
    
    // Pre-populate camera info message templates
    setupCameraInfoTemplates();
    this->last_image_time=std::chrono::steady_clock::now();

    // start the publishing loop
    this->timer = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&ZedBridge::publishImages, this));
}

void ZedBridge::setupCameraInfoTemplates() {
    // Set up left camera info template
    left_camera_info_template.distortion_model = "plumb_bob";
    left_camera_info_template.d.resize(5);
    left_camera_info_template.d[0] = cached_calibration_params.left_cam.disto[0];
    left_camera_info_template.d[1] = cached_calibration_params.left_cam.disto[1];
    left_camera_info_template.d[2] = cached_calibration_params.left_cam.disto[2];
    left_camera_info_template.d[3] = cached_calibration_params.left_cam.disto[3];
    left_camera_info_template.d[4] = cached_calibration_params.left_cam.disto[4];
    left_camera_info_template.k.fill(0.0);
    left_camera_info_template.k[0] = cached_calibration_params.left_cam.fx;
    left_camera_info_template.k[2] = cached_calibration_params.left_cam.cx;
    left_camera_info_template.k[4] = cached_calibration_params.left_cam.fy;
    left_camera_info_template.k[5] = cached_calibration_params.left_cam.cy;
    left_camera_info_template.k[8] = 1.0;
    left_camera_info_template.p.fill(0.0);
    left_camera_info_template.p[0] = cached_calibration_params.left_cam.fx;
    left_camera_info_template.p[2] = cached_calibration_params.left_cam.cx;
    left_camera_info_template.p[5] = cached_calibration_params.left_cam.fy;
    left_camera_info_template.p[6] = cached_calibration_params.left_cam.cy;
    left_camera_info_template.p[10] = 1.0;

    // Copy for depth camera info template
    depth_camera_info_template = left_camera_info_template;
}

void ZedBridge::publishImages() {
    auto err = zed.grab(this->runtime_parameters);

    if (err == ERROR_CODE::SUCCESS) {
        // Cache timestamp once
        const auto timestamp = this->now();

        // retrieve the left image
        sl::Mat left_image;
        zed.retrieveImage(left_image, VIEW::LEFT);

        //Measure latency
        auto start_time = std::chrono::high_resolution_clock::now();

        zed.retrieveObjects(this->objects, this->obj_runtime_param);
        sl::Mat point_cloud;
        zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZ);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // Pre-allocate reusable objects as static to avoid repeated allocations
        static cv::Mat left_image_cv;
        static sensor_msgs::msg::Image left_image_msg;
        static sensor_msgs::msg::Image depth_image_msg;
        static lart_msgs::msg::ConeArray cone_array;
        static visualization_msgs::msg::MarkerArray marker_array;

        // Clear containers instead of recreating
        cone_array.cones.clear();
        marker_array.markers.clear();

        // convert the image to OpenCV format
        left_image_cv = slMat2cvMat(left_image);
        cv::cvtColor(left_image_cv, left_image_cv, cv::COLOR_BGRA2BGR);

        // convert the image to a ROS message
        left_image_msg.header.stamp = timestamp;
        left_image_msg.header.frame_id = LEFT_IMG_FRAME_ID;
        left_image_msg.height = left_image_cv.rows;
        left_image_msg.width = left_image_cv.cols;
        left_image_msg.encoding = "bgr8";
        left_image_msg.step = left_image_cv.step;
        
        // More efficient image data copying
        left_image_msg.data.assign(left_image_cv.data, 
                                   left_image_cv.data + left_image_cv.rows * left_image_cv.cols * left_image_cv.channels());

        // Reserve space for known maximum objects
        cone_array.cones.reserve(objects.object_list.size());
        marker_array.markers.reserve(objects.object_list.size() + this->marker_ids.size());

        //eliminate old markers
        for (const auto& marker_id : this->marker_ids) {
            visualization_msgs::msg::Marker old_marker;
            old_marker.header.frame_id = "base_footprint";
            old_marker.header.stamp = timestamp;
            old_marker.ns = "cone_marker";
            old_marker.id = marker_id;
            old_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(std::move(old_marker));
        }

        this->marker_ids.clear();

        // Optimized transform lookup with caching
        static geometry_msgs::msg::TransformStamped cached_transform;
        static auto last_transform_time = std::chrono::steady_clock::now();
        
        auto now = std::chrono::steady_clock::now();
        if (now - last_transform_time > std::chrono::milliseconds(100)) {
            try {
                cached_transform = this->tf_buffer->lookupTransform("base_footprint", "zed_camera_center", rclcpp::Time(0));
                last_transform_time = now;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not get transform: %s", ex.what());
            }
        }

        // Optimized object processing loop
        for (size_t i = 0; i < objects.object_list.size(); ++i) {
            const auto& obj = objects.object_list[i];
            
            // Filter early to avoid unnecessary processing
            if (std::isnan(obj.position.x) || std::isinf(obj.position.x)) {
                continue;
            }

            // Use squared distance to avoid sqrt
            const float distance_sq = obj.position.x * obj.position.x + obj.position.y * obj.position.y;
            
            // --- NEW PINHOLE LOGIC START  - Code added by Ian ---

            // 1. Get Height of the bounding box in Pixels
            float height_px = 0.0f;
            if (obj.bounding_box_2d.size() >= 4) {
                float top_y = static_cast<float>(obj.bounding_box_2d[0].y);
                float bottom_y = static_cast<float>(obj.bounding_box_2d[2].y);
                height_px = bottom_y - top_y;
            }

            // 2. Get Focal Length
            float fy = this->cached_calibration_params.left_cam.fy;

            // 3. Calculate Pinhole Distance
            float pinhole_dist = 0.0f;
            float cone_real_height = 0.325f; // 32.5cm

            if (height_px > 0.0f) {
                pinhole_dist = (fy * cone_real_height) / height_px;
            }

            // 4. Calculate ZED Distance
            float zed_dist = std::sqrt(distance_sq);

            RCLCPP_INFO(this->get_logger(), "Object Calculated Distance : %.2f Confidence: %.2f", zed_dist, obj.confidence);

            // 5. Print the comparison
            RCLCPP_INFO(this->get_logger(), 
                "Comparison between distances calculated by ZED and by bounding box size:  H_px: %.0f | ZED: %.2fm | Pinhole: %.2fm | Diff: %.2fm", 
                height_px, zed_dist, pinhole_dist, std::abs(zed_dist - pinhole_dist));

            // --- NEW PINHOLE LOGIC END ---

            if (distance_sq < 0.25f || distance_sq > 650.0f) { // 0.5^2 = 0.25, 20^2 = 400
                continue;
            }

            const float obj_x = obj.position.x;
            const float obj_y = obj.position.y;
            const float obj_z = obj.position.z;

            // Cache transformed positions
            const float transformed_x = transform_matrix[0][0] * obj_x + transform_matrix[0][1] * obj_y + 
                                      transform_matrix[0][2] * obj_z + transform_matrix[0][3];
            const float transformed_y = transform_matrix[1][0] * obj_x + transform_matrix[1][1] * obj_y + 
                                      transform_matrix[1][2] * obj_z + transform_matrix[1][3];
            const float transformed_z = transform_matrix[2][0] * obj_x + transform_matrix[2][1] * obj_y + 
                                      transform_matrix[2][2] * obj_z + transform_matrix[2][3];

            //create cone message
            lart_msgs::msg::Cone cone;
            cone.header.frame_id = "base_footprint";
            cone.position.x = transformed_x;
            cone.position.y = transformed_y;
            cone.position.z = 0.0;
            cone.class_type.data = obj.raw_label;

            //add cone to array
            cone_array.cones.push_back(std::move(cone));

            //create marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = timestamp;
            marker.ns = "cone_marker";
            marker.id = this->frame_counter * 1000 + i;
            this->marker_ids.push_back(marker.id);

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration::from_seconds(0.2); // Increased from 0.1 to reduce recreation overhead

            marker.pose.position.x = transformed_x;
            marker.pose.position.y = transformed_y;
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
            
            marker_array.markers.push_back(std::move(marker));
        }

        //publish cone array and markers using move semantics
        this->cone_array_pub->publish(std::move(cone_array));
        this->marker_array_pub->publish(std::move(marker_array));

        // retrieve the depth image
        sl::Mat depth_image;
        zed.retrieveMeasure(depth_image, MEASURE::DEPTH);

        // convert the depth image to a ROS message
        depth_image_msg.header.stamp = timestamp;
        depth_image_msg.header.frame_id = LEFT_IMG_FRAME_ID;
        depth_image_msg.height = depth_image.getHeight();
        depth_image_msg.width = depth_image.getWidth();
        depth_image_msg.encoding = "32FC1"; // 32-bit float for depth
        depth_image_msg.step = depth_image.getStepBytes();
        const size_t data_size = depth_image.getHeight() * depth_image.getWidth() * sizeof(float);
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(depth_image.getPtr<sl::float1>());
        depth_image_msg.data.assign(data_ptr, data_ptr + data_size);

        // Use cached camera info templates
        left_camera_info_template.header.stamp = timestamp;
        left_camera_info_template.height = left_image_msg.height;
        left_camera_info_template.width = left_image_msg.width;
        left_camera_info_template.header.frame_id = LEFT_IMG_FRAME_ID;

        depth_camera_info_template.header.stamp = timestamp;
        depth_camera_info_template.height = depth_image_msg.height;
        depth_camera_info_template.width = depth_image_msg.width;
        depth_camera_info_template.header.frame_id = LEFT_IMG_FRAME_ID;

        // publish the images using move semantics
        this->left_image_pub.publish(std::make_shared<sensor_msgs::msg::Image>(std::move(left_image_msg)));
        this->depth_image_pub.publish(std::make_shared<sensor_msgs::msg::Image>(std::move(depth_image_msg)));

        // publish the camera info using cached templates
        left_info_pub->publish(left_camera_info_template);
        depth_info_pub->publish(depth_camera_info_template);

        this->frame_counter++;
        this->last_image_time = std::chrono::steady_clock::now();
        this->first_image = true;
    }else if (err != ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "ZED camera grab failed");
        // if (this->first_image){
            if ( std::chrono::steady_clock::now() - this->last_image_time > std::chrono::seconds(1)) {
                RCLCPP_ERROR(this->get_logger(), "ZED camera not responding, publishing emergency state");
                this->first_image = false;
                lart_msgs::msg::State emergency;
                emergency.data = lart_msgs::msg::State::EMERGENCY;
                this->emergency_pub->publish(emergency);
            }
        // }
    }
}

void ZedBridge::transformListener(const geometry_msgs::msg::TransformStamped & transform) {
    // This function is not used in this implementation, but can be used to listen to transforms
    // if needed in the future.
    RCLCPP_INFO(this->get_logger(), "Received transform from %s to %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
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
