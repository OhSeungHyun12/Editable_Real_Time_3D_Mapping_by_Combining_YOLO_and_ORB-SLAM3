#include "headers.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace std;

std::mutex mtx;
std::condition_variable cv_frame_ready;
cv::Mat sharedFrame;
double sharedTimestamp;
bool newFrameAvailable = false;
std::atomic<bool> gl_stop_sig(false);                  // Global termination signal flag
std::atomic<int>  gl_slam_state(0);                    // ORB-SLAM3 Tracking state (1/2/3)
std::atomic<bool> gl_map_points(false);                // If a map point exists -> true

void YoloThread(YoloDetection* yolo,
                rclcpp::Node::SharedPtr node,
                rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub,
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub)
{
    while (rclcpp::ok() && !gl_stop_sig.load()) {
        cv::Mat frame;
        double timestamp;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame_ready.wait(lock, [] { return newFrameAvailable || gl_stop_sig.load(); });
            if (gl_stop_sig.load()) break;

            frame = sharedFrame.clone();
            timestamp = sharedTimestamp;
            newFrameAvailable = false;
        }
        
        if (!frame.empty()) {
            yolo->GetImage(frame);
            bool has_det = (!yolo->mRGB.empty() && yolo->Detect());

            vision_msgs::msg::Detection2DArray detec_msg;
            detec_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
            detec_msg.header.frame_id = "camera";                           // Camera coordinate system name
        
            if (has_det) {
                for (const auto& det : yolo->mvDetections) {
                    vision_msgs::msg::Detection2D det_msg;                  
                    det_msg.header = detec_msg.header;
                    
                    // Set the center and size of the bounding box
                    det_msg.bbox.center.position.x = det.box.x + det.box.width * 0.5;
                    det_msg.bbox.center.position.y = det.box.y + det.box.height * 0.5;
                    det_msg.bbox.size_x = det.box.width;
                    det_msg.bbox.size_y = det.box.height;

                    // Detection results (class name, confidence)
                    vision_msgs::msg::ObjectHypothesisWithPose hyp;         // hyp = hypothesis
                    hyp.hypothesis.class_id = det.label;
                    hyp.hypothesis.score    = det.confidence;
                    det_msg.results.push_back(hyp);
                
                    detec_msg.detections.push_back(det_msg);
                }                
            }

            pub->publish(detec_msg);
            
            if (overlay_pub) {
                cv::Mat viz = frame.clone();
                
                if (has_det) {
                    for (const auto& d : yolo->mvDetections) {
                        cv::rectangle(viz, d.box, cv::Scalar(0,255,0), 2);
                        std::string text = d.label + " " + cv::format("%.2f", d.confidence);
                        cv::putText(viz, text, d.box.tl() + cv::Point(0,-5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
                    }
                }
                
                int st = gl_slam_state.load(std::memory_order_relaxed);
                bool ok = (st == 2) && gl_map_points.load(std::memory_order_relaxed);
                std::string s;
                cv::Scalar col;
                
                if (st == 3) {
                    s = "SLAM LOST"; col = cv::Scalar(0,0,255);
                } else if (ok) {
                    s = "SLAM ON";   col = cv::Scalar(0,255,0);
                } else {
                    s = "SLAM OFF";  col = cv::Scalar(0,0,255);
                }
                
                cv::putText(viz, s, cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, col, 2);

                auto img_msg = cv_bridge::CvImage(detec_msg.header, "bgr8", viz).toImageMsg();
                overlay_pub->publish(*img_msg);
            }
        }
    }
    std::cout << "YOLO thread finished" << std::endl;
}

class SlamNode : public rclcpp::Node
{
public:
    SlamNode(ORB_SLAM3::System* pSLAM, std::ofstream& trajectory_file)
    : Node("slam_node"), mpSLAM(pSLAM), mTrajectoryFile(trajectory_file)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&SlamNode::GrabImage, this, std::placeholders::_1));

        mPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);
        mPointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        mTimer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SlamNode::PublishMapAndPose, this));

        mTrajectoryFile << "# Camera Trajectory" << endl;
        mTrajectoryFile << "# timestamp tx ty tz qx qy qz qw" << endl;
    }

private:
    
    void PublishMapAndPose()                            // 3) Functions that generate poses and point clouds
    {
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp = this->get_clock()->now();
        poseMsg.header.frame_id = "map";                // Reference of coordinate system "map->(you can change)"

        {
            // Pose Publish & TF
            std::lock_guard<std::mutex> lock(mPoseMutex);
            if (!mLastPose.unit_quaternion().isApprox(Eigen::Quaternionf(0,0,0,0))) {
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

                if (tf_broadcaster_) {
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header = poseMsg.header;
                    tf_msg.child_frame_id = "camera";     
                    tf_msg.transform.translation.x = p.x();
                    tf_msg.transform.translation.y = p.y();
                    tf_msg.transform.translation.z = p.z();
                    tf_msg.transform.rotation.x = q.x();
                    tf_msg.transform.rotation.y = q.y();
                    tf_msg.transform.rotation.z = q.z();
                    tf_msg.transform.rotation.w = q.w();
                    tf_broadcaster_->sendTransform(tf_msg);
                }
            }
        }

        const auto& all = mpSLAM->GetAllMapPoints();
        
        std::vector<Eigen::Vector3f> pts;
        pts.reserve(all.size());
        for (auto* pMP : all) {
            if (pMP && !pMP->isBad()) pts.push_back(pMP->GetWorldPos());
        }

        gl_map_points.store(!pts.empty(), std::memory_order_relaxed);
        if (pts.empty()) return;

        sensor_msgs::msg::PointCloud2 cloudMsg;
        cloudMsg.header.stamp = this->get_clock()->now();
        cloudMsg.header.frame_id = "map";
        cloudMsg.height = 1; 
        cloudMsg.width = pts.size();
        cloudMsg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(pts.size());

        sensor_msgs::PointCloud2Iterator<float> ix_map(cloudMsg, "x");      // PointCloud2 iterators to write XYZ in the 'map' frame.
        sensor_msgs::PointCloud2Iterator<float> iy_map(cloudMsg, "y");      // RViz uses TF (map → camera, etc.) only for display.
        sensor_msgs::PointCloud2Iterator<float> iz_map(cloudMsg, "z");

        for (const auto& P : pts) {
            *ix_map = P.x(); *iy_map = P.y(); *iz_map = P.z();
            ++ix_map; ++iy_map; ++iz_map;
        }
        mPointCloudPub->publish(cloudMsg);
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image.clone();
        double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Sophus::SE3f Tcw = mpSLAM->TrackMonocular(frame, tframe);
        gl_slam_state.store(mpSLAM->GetTrackingState(), std::memory_order_relaxed);


        //===== Start camera trajectory saving logic =====//
        // 4) Latest Pose -> member variable
        if (!Tcw.unit_quaternion().isApprox(Eigen::Quaternionf(0,0,0,0))) {       
            std::lock_guard<std::mutex> lock(mPoseMutex);
            mLastPose = Tcw.inverse();

            // Calculate camera pose (Twc) relative to world coordinates
            Sophus::SE3f Twc = Tcw.inverse();

            // Translation and Quaternion Extraction
            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            // TUM format: timestamp tx ty tz qx qy qz qw
            mTrajectoryFile << fixed << setprecision(7) << tframe << " "
                            << twc.x() << " " << twc.y() << " " << twc.z() << " "
                            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        //================================================//

        // Passing to YOLO thread
        {
            std::lock_guard<std::mutex> lock(mtx);
            sharedFrame = frame.clone();
            sharedTimestamp = tframe;
            newFrameAvailable = true;
        }
        cv_frame_ready.notify_one();
    }

    ORB_SLAM3::System* mpSLAM;
    std::ofstream& mTrajectoryFile;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPosePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPub;
    rclcpp::TimerBase::SharedPtr mTimer;
    Sophus::SE3f mLastPose;
    std::mutex mPoseMutex;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);  // ORB default viewer is OFF (to avoid conflicts with Pangolin windows)
    YoloDetection yolo;

    auto slam_node = std::make_shared<SlamNode>(&SLAM, trajectoryFile);

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
    
    cout << "Shutdown complete." << endl;
    return 0;
}
