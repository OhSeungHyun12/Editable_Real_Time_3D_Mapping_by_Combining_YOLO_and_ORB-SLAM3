#include "headers.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

std::mutex mtx_frame;
std::condition_variable cv_frame_ready;
cv::Mat sharedFrame;
double sharedTimestamp;
bool newFrameAvailable = false;                                                                 // If a map point exists -> true

std::mutex mtx_overlay_pts;                                                                     // 2D overlay point
std::vector<std::pair<cv::Point2f, uint32_t>> overlay_pts;

std::atomic<bool> gl_stop_sig(false);                                                           // Global termination signal flag
std::atomic<int>  gl_slam_state(0);                                                             // ORB-SLAM3 Tracking state (1/2/3)

void YoloThread(YoloDetection* yolo,
                rclcpp::Node::SharedPtr node,
                rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub,
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub)
{
    while (rclcpp::ok() && !gl_stop_sig.load()) {
        cv::Mat frame;
        double timestamp;
        {
            std::unique_lock<std::mutex> lock(mtx_frame);
            cv_frame_ready.wait(lock, [] { return newFrameAvailable || gl_stop_sig.load(); });
            if (gl_stop_sig.load()) break;
            frame = sharedFrame.clone();
            timestamp = sharedTimestamp;
            newFrameAvailable = false;
        }
        
        if (frame.empty()) continue;

        yolo->GetImage(frame);
        bool has_det = yolo->Detect();

        // Convert YOLO detection results into ROS messages and publish
        vision_msgs::msg::Detection2DArray detec_msg;
        detec_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
        detec_msg.header.frame_id = "camera";                                               // Camera coordinate system name
        
        if (has_det) {
            for (const auto& det : yolo->GetDetections()) {
                vision_msgs::msg::Detection2D det_msg;
                det_msg.header = detec_msg.header;
                    
                // Set the center and size of the bounding box
                det_msg.bbox.center.position.x = det.box.x + det.box.width * 0.5;
                det_msg.bbox.center.position.y = det.box.y + det.box.height * 0.5;
                det_msg.bbox.size_x = det.box.width;
                det_msg.bbox.size_y = det.box.height;

                // Detection results (class name, confidence)
                vision_msgs::msg::ObjectHypothesisWithPose hyp;                             // hyp = hypothesis
                hyp.hypothesis.class_id = det.label;
                hyp.hypothesis.score = det.confidence;
                det_msg.results.push_back(hyp);
                detec_msg.detections.push_back(det_msg);
            }
        }
        pub->publish(detec_msg);
        
        if (overlay_pub) {
            cv::Mat viz = frame.clone();
            if (has_det) {
                for (const auto& d : yolo->GetDetections()) {
                    uint32_t color = yolo->GetColorForClass(d.class_id);
                    uint8_t b = ( color ) & 0xFF;
                    uint8_t g = ( color >> 8 ) & 0xFF;
                    uint8_t r = ( color >> 16 ) & 0xFF;
                    cv::Scalar box_color(b, g, r);
                    cv::rectangle(viz, d.box, box_color, 2);
                    std::string text = d.label + " " + cv::format("%.2f", d.confidence);
                    cv::putText(viz, text, d.box.tl() + cv::Point(0, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1);
                }
            }

            {
                std::lock_guard<std::mutex> lk(mtx_overlay_pts);
                for(const auto& pr : overlay_pts) {
                    const cv::Point2f& uv = pr.first;
                    uint32_t rgb = pr.second;
                    // unpack 0x00BBGGRR → B,G,R
                    uint8_t b = (rgb) & 0xFF, g = (rgb >> 8) & 0xFF, r = (rgb >> 16) & 0xFF;
                    // small square dot (2-3px) or circle
                    cv::circle(viz, uv, 2, cv::Scalar(b,g,r), cv::FILLED, cv::LINE_AA);
                }
            }
            int st = gl_slam_state.load(std::memory_order_relaxed);
            std::string s;
            cv::Scalar col;
            
            if ( st == 3 ) {
                s = "SLAM LOST"; col = cv::Scalar( 0,0,255 );
            } else if ( st == 2 ) {
                s = "SLAM ON";   col = cv::Scalar( 0,255,0 );
            } else {
                s = "SLAM OFF";  col = cv::Scalar( 0,0,255 );
            }
            cv::putText( viz, s, cv::Point( 12, 28 ), cv::FONT_HERSHEY_SIMPLEX, 0.9, col, 2 );
            
            auto img_msg = cv_bridge::CvImage(detec_msg.header, "bgr8", viz).toImageMsg();
            overlay_pub->publish(*img_msg);
        }
    }
    std::cout << "YOLO thread finished" << std::endl;
}

class SlamNode : public rclcpp::Node
{
public:
    SlamNode(ORB_SLAM3::System* pSLAM, YoloDetection* pYOLO, std::ofstream& trajectory_file)
    : Node("slam_node"), mpSLAM(pSLAM), mpYOLO(pYOLO), mTrajectoryFile(trajectory_file)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&SlamNode::GrabImage, this, std::placeholders::_1));
        
        det_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>( 
            "/yolo_detections", 10, [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->det_mtx_);
            this->last_dets_ = msg;
        });            
        mPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);
        mMapPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_map", 10);
        static_cloud_OctoMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/OctoMap_pointcloud", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        mTimer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SlamNode::PublishData, this));        
    }

private:

    // --- SLAM / YOLO ---
    ORB_SLAM3::System* mpSLAM;
    YoloDetection* mpYOLO;

    // --- ROS IO ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPosePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapPub;                      // semantic_points
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_OctoMap;         // ★ /OctoMap_pointcloud
    rclcpp::TimerBase::SharedPtr mTimer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Pose / Trajectory ---
    std::ofstream& mTrajectoryFile;
    Sophus::SE3f mLastPose;
    std::mutex mPoseMutex;

    // --- Semantic label map (MapPoint* -> class_id) ---
    std::unordered_map<size_t, int> mp_semantic_label_;

    // --- Detections cache ---
    vision_msgs::msg::Detection2DArray::SharedPtr last_dets_;
    std::mutex det_mtx_;

    // --- 슬라이딩 윈도우를 위한 deque ---
    size_t overlay_window_size_ = 3;
    std::deque<std::unordered_map<size_t, int>> m_semantic_hist;

    void PublishData() {
        PublishPoseAndTF();
        LabelMapPoints();
        PublishVisualizationMap();
        PublishOctomapInput();
    }
    
    void PublishPoseAndTF() {
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp = this->get_clock()->now();
        poseMsg.header.frame_id = "map";

        std::lock_guard<std::mutex> lock(mPoseMutex);
        if (mLastPose.unit_quaternion().isApprox(Eigen::Quaternionf(0,0,0,0))) return;
        
        Eigen::Vector3f p = mLastPose.translation();
        Eigen::Quaternionf q = mLastPose.unit_quaternion();

        poseMsg.pose.position.x = p.x();
        poseMsg.pose.position.y = p.y();
        poseMsg.pose.position.z = p.z();
        poseMsg.pose.orientation.x = q.x();
        poseMsg.pose.orientation.y = q.y();
        poseMsg.pose.orientation.z = q.z();
        poseMsg.pose.orientation.w = q.w();
        mPosePub->publish(poseMsg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = poseMsg.header;
        tf_msg.child_frame_id = "camera";     
        tf_msg.transform.translation.x = p.x();
        tf_msg.transform.translation.y = p.y();
        tf_msg.transform.translation.z = p.z();
        tf_msg.transform.rotation = poseMsg.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void LabelMapPoints() {
        m_semantic_hist.push_back({});                          // 새 프레임을 위한 빈 맵 추가
        if(m_semantic_hist.size() > overlay_window_size_) {
            m_semantic_hist.pop_front();                        // 윈도우 크기 유지
        }
        auto& current_labels = m_semantic_hist.back();

        vision_msgs::msg::Detection2DArray::SharedPtr dets;
        {
            std::lock_guard<std::mutex> lk(det_mtx_);
            dets = last_dets_;
        }
        if(!dets || dets->detections.empty()) return;
        
        auto kpsUn = mpSLAM->GetTrackedKeyPointsUn();
        auto mps = mpSLAM->GetTrackedMapPoints();
        if(mps.empty() || kpsUn.empty()) return;

        std::vector<std::pair<cv::Point2f, uint32_t>> local_overlay_pts;

        for (size_t i = 0; i < kpsUn.size() && i < mps.size(); ++i) {
            auto* pMP = mps[i];
            if (!pMP || pMP->isBad()) continue;
            
            const auto& kp = kpsUn[i];
            for (const auto& det : dets->detections) {
                const auto& b = det.bbox;
                float x0 = b.center.position.x - b.size_x * 0.5f;
                float y0 = b.center.position.y - b.size_y * 0.5f;
                float x1 = x0 + b.size_x;
                float y1 = y0 + b.size_y;

                if (kp.pt.x >= x0 && kp.pt.x <= x1 && kp.pt.y >= y0 && kp.pt.y <= y1) {
                    if (det.results.empty()) continue;

                    int class_id = -1;
                    const auto& class_names = mpYOLO->GetClassNames();
                    const std::string& label = det.results[0].hypothesis.class_id;
                    auto it = std::find(class_names.begin(), class_names.end(), label);
                    if (it != class_names.end()) {
                        class_id = std::distance(class_names.begin(), it);
                    }

                    if (class_id != -1) {
                        size_t key = reinterpret_cast<size_t>(pMP);
                        current_labels[key] = class_id;
                        uint32_t rgb = mpYOLO->GetColorForClass(class_id);
                        local_overlay_pts.emplace_back(kp.pt, rgb);
                    }
                    break;
                }
            }
        }

        if (!local_overlay_pts.empty()) {
            std::lock_guard<std::mutex> lk(mtx_overlay_pts);
            overlay_pts = local_overlay_pts;
        } else {
            std::lock_guard<std::mutex> lk(mtx_overlay_pts);
            overlay_pts.clear();
        }
    }

    void PublishVisualizationMap() {
        const auto& all = mpSLAM->GetAllMapPoints();
        if(all.empty()) return;

        struct PointData { Eigen::Vector3f pos; uint32_t rgb; };
        std::vector<PointData> buf;
        buf.reserve(all.size());

        uint32_t white_color = (255) | (255 << 8) | (255 << 16);

        // 최근 라벨링된 맵포인트를 저장하는 임시 맵
        std::unordered_map<size_t, int> recent_labels;
        for(const auto& hist_map : m_semantic_hist) {
            recent_labels.insert(hist_map.begin(), hist_map.end());
        }

        for(auto* pMP : all) {
            if (!pMP || pMP->isBad()) continue;
            
            size_t key = reinterpret_cast<size_t>(pMP);
            auto it = recent_labels.find(key);        
            uint32_t rgb = (it != recent_labels.end()) ? mpYOLO->GetColorForClass(it->second) : white_color;
            buf.push_back({ pMP->GetWorldPos(), rgb });
        }
        
        // --- 시각화용 /semantic_points (XYZRGB) ---
        sensor_msgs::msg::PointCloud2 cloud_vis;
        cloud_vis.header.stamp = this->get_clock()->now();
        cloud_vis.header.frame_id = "map";        
        sensor_msgs::PointCloud2Modifier mod_vis(cloud_vis);
        mod_vis.setPointCloud2FieldsByString(2, "xyz", "rgb");
        mod_vis.resize(buf.size());
        sensor_msgs::PointCloud2Iterator<float> ix(cloud_vis,"x"), iy(cloud_vis,"y"), iz(cloud_vis,"z");
        sensor_msgs::PointCloud2Iterator<uint8_t> i_rgb(cloud_vis, "rgb");
        for(const auto& point : buf) {
            *ix = point.pos.x(); *iy = point.pos.y(); *iz = point.pos.z();
            std::memcpy(&(*i_rgb), &point.rgb, 4);
            ++ix; ++iy; ++iz; ++i_rgb;
        }
        mMapPub->publish(cloud_vis);
    }

    void PublishOctomapInput()
    {
        // 현재 프레임에서 추적된 포인트들만 사용
        auto mps = mpSLAM->GetTrackedMapPoints();
        if(mps.empty() || m_semantic_hist.empty()) return;

        // 현재 프레임의 라벨 정보만 가져옴
        const auto& current_labels = m_semantic_hist.back();
    
        // 현재 카메라의 Pose (World -> Camera)
        Sophus::SE3f Tcw;
        {
            std::lock_guard<std::mutex> lock(mPoseMutex);
            Tcw = mLastPose.inverse();
        }
        if (Tcw.unit_quaternion().isApprox(Eigen::Quaternionf(0,0,0,0))) return;

        std::vector<Eigen::Vector3f> static_points_camera_frame;
        static_points_camera_frame.reserve(mps.size());

        for(auto* pMP : mps) {
            if (!pMP || pMP->isBad()) continue;

            size_t key = reinterpret_cast<size_t>(pMP);
            // 현재 프레임에서 객체로 라벨링되지 않은 포인트만 선택
            if (current_labels.find(key) == current_labels.end()) {
                // 월드 좌표계의 포인트를 카메라 좌표계로 변환
                Eigen::Vector3f p_cam = Tcw * pMP->GetWorldPos();
                static_points_camera_frame.push_back(p_cam);
            }
        }
        
        if (static_points_camera_frame.empty()) return;
        
        // --- PointCloud2 생성 및 발행 ---
        sensor_msgs::msg::PointCloud2 cloud_static;
        cloud_static.header.stamp = this->get_clock()->now();
        cloud_static.header.frame_id = "camera";

        sensor_msgs::PointCloud2Modifier mod_static(cloud_static);
        mod_static.setPointCloud2FieldsByString(1, "xyz");
        mod_static.resize(static_points_camera_frame.size());

        sensor_msgs::PointCloud2Iterator<float> sx(cloud_static,"x"), sy(cloud_static,"y"), sz(cloud_static,"z");
        for(const auto& p : static_points_camera_frame) {
            *sx = p.x(); *sy = p.y(); *sz = p.z();
            ++sx; ++sy; ++sz;
        }
        static_cloud_OctoMap->publish(cloud_static);
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image.clone();
        double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Sophus::SE3f Tcw = mpSLAM->TrackMonocular(frame, tframe);
        gl_slam_state.store(mpSLAM->GetTrackingState(), std::memory_order_relaxed);

        if (!Tcw.unit_quaternion().isApprox(Eigen::Quaternionf(0,0,0,0))) {       
            std::lock_guard<std::mutex> lock(mPoseMutex);
            mLastPose = Tcw.inverse();
            
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            mTrajectoryFile << std::fixed << std::setprecision(7) << tframe << " "
                            << twc.x() << " " << twc.y() << " " << twc.z() << " "
                            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(mtx_frame);
            sharedFrame = frame.clone();
            sharedTimestamp = tframe;
            newFrameAvailable = true;
        }
        cv_frame_ready.notify_one();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "\nUsage: ros2 run yolo_orb3_ros2 mono_ar path_to_vocabulary path_to_settings\n";
        return 1;
    }

    //==== Creating and initializing CameraTrajectory.txt ====//
    std::ofstream trajectoryFile;
    trajectoryFile.open("CameraTrajectory.txt", std::ios::out | std::ios::trunc);
    if (!trajectoryFile.is_open()) {
        std::cerr << "FATAL: Could not open CameraTrajectory.txt for writing.\n";
        return 1;
    }
    //========================================================//

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
    YoloDetection yolo;

    auto slam_node = std::make_shared<SlamNode>(&SLAM, &yolo, trajectoryFile);

    // Create YOLO Publisher on the main node
    auto yolo_pub = slam_node->create_publisher<vision_msgs::msg::Detection2DArray>("/yolo_detections", 10);
    auto overlay_pub = slam_node->create_publisher<sensor_msgs::msg::Image>("/yolo/image_overlay", 10);

    // Start YoloThread
    std::thread yolo_thread(YoloThread, &yolo, slam_node, yolo_pub, overlay_pub);

    rclcpp::spin(slam_node);

    gl_stop_sig.store(true);
    cv_frame_ready.notify_all();
    yolo_thread.join();

    SLAM.Shutdown();
    trajectoryFile.close();
    rclcpp::shutdown();
    
    std::cout << "Shutdown complete." << std::endl;
    return 0;
}
