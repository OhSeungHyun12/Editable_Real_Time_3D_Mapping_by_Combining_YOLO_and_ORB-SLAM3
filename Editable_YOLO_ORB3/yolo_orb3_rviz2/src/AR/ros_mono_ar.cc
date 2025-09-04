#include "headers.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace std;

std::mutex mtx;
std::condition_variable cv_frame_ready;
cv::Mat sharedFrame;
double sharedTimestamp;
bool newFrameAvailable = false;
std::atomic<bool> g_stop_signal(false);                 // Global termination signal flag

void YoloThread(YoloDetection* yolo, rclcpp::Node::SharedPtr node, rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub)
{
    auto clock = node->get_clock();
    while (rclcpp::ok() && !g_stop_signal.load()) {
        cv::Mat frame;
        double timestamp;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame_ready.wait(lock, [] { return newFrameAvailable || g_stop_signal.load(); });

            if (g_stop_signal.load()) break;

            frame = sharedFrame.clone();
            timestamp = sharedTimestamp;
            newFrameAvailable = false;
        }

        if (!frame.empty()) {
            yolo->GetImage(frame);
            if (!yolo->mRGB.empty() && yolo->Detect()) {
                vision_msgs::msg::Detection2DArray detections_msg;
                detections_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
                detections_msg.header.frame_id = "camera"; // Camera coordinate system name

                for (const auto& det : yolo->mvDetections) {
                    vision_msgs::msg::Detection2D detection;
                    detection.header = detections_msg.header;
                    
                    // Set the center and size of the bounding box
                    detection.bbox.center.position.x = det.box.x + det.box.width / 2.0;
                    detection.bbox.center.position.y = det.box.y + det.box.height / 2.0;
                    detection.bbox.size_x = det.box.width;
                    detection.bbox.size_y = det.box.height;

                    // Detection results (class name, confidence)
                    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                    hypothesis.hypothesis.class_id = det.label;
                    hypothesis.hypothesis.score = det.confidence;
                    detection.results.push_back(hypothesis);

                    detections_msg.detections.push_back(detection);
                }
                pub->publish(detections_msg);
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
        
        mTimer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SlamNode::PublishMapAndPose, this));

        mTrajectoryFile << "# Camera Trajectory" << endl;
        mTrajectoryFile << "# timestamp tx ty tz qx qy qz qw" << endl;
    }

private:
    
    void PublishMapAndPose()                            // 3) Functions that generate poses and point clouds
    {
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp = this->get_clock()->now();
        poseMsg.header.frame_id = "map";                // Reference of coordinate system "___"

        {
            // Protecting mLastPose Access with a Mutex
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
            }
        }

        std::vector<ORB_SLAM3::MapPoint*> vMPs = mpSLAM->GetAllMapPoints();
        if(vMPs.empty()) return;

        sensor_msgs::msg::PointCloud2 cloudMsg;
        cloudMsg.header.stamp = this->get_clock()->now();
        cloudMsg.header.frame_id = "map";
        cloudMsg.height = 1; 
        cloudMsg.width = vMPs.size();
        cloudMsg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(vMPs.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloudMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloudMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloudMsg, "z");

        for (ORB_SLAM3::MapPoint* pMP : vMPs) {
            if (pMP && !pMP->isBad()) {
                Eigen::Vector3f pos = pMP->GetWorldPos();
                *iter_x = pos.x();
                *iter_y = pos.y();
                *iter_z = pos.z();
                ++iter_x; ++iter_y; ++iter_z;
            }
        }
        mPointCloudPub->publish(cloudMsg);
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image.clone();
        double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Sophus::SE3f Tcw = mpSLAM->TrackMonocular(frame, tframe);

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

    // Start YoloThread
    std::thread yolo_thread(YoloThread, &yolo, slam_node, yolo_pub);

    rclcpp::spin(slam_node);

    g_stop_signal.store(true);
    cv_frame_ready.notify_all();
    yolo_thread.join();

    SLAM.Shutdown();
    trajectoryFile.close();
    rclcpp::shutdown();
    
    cout << "Shutdown complete." << endl;
    return 0;
}
