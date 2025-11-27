#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

const double MARKER_LENGTH = 0.065; // meter

// 世界中每個 marker 的中心位置 (X,Y,Z)
struct MarkerInfo {
    int id;
    cv::Point3f pos;
};
const std::vector<MarkerInfo> WORLD_MARKERS = {
    {20, {0.6f, 1.4f, 0.0f}},
    {21, {2.4f, 1.4f, 0.0f}},
    {22, {0.6f, 0.6f, 0.0f}},
    {23, {2.4f, 0.6f, 0.0f}},
};

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_detector_node")
    {
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ArucoDetectorNode::image_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.1;
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 10;
        detector_params_->minCornerDistanceRate = 0.05;
        detector_params_->minMarkerDistanceRate = 0.05;



        camera_matrix_ = (cv::Mat1d(3, 3) <<
            605.9448852539062, 0.0, 314.1728515625,
            0.0, 605.9448852539062, 247.0227813720703,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        RCLCPP_INFO(this->get_logger(), "Aruco solvePnPRansac version started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);
        if (ids.empty()) {
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(1);
            return;
        }

        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        RCLCPP_INFO(this->get_logger(), "Detected %zu markers.", ids.size());

        // --- 建立 solvePnPRansac 所需的點對 ---
        std::vector<cv::Point3f> objectPoints; // world 3D
        std::vector<cv::Point2f> imagePoints;  // image 2D

        auto add_marker = [&](int id, cv::Point3f center, float L) {
            auto it = std::find(ids.begin(), ids.end(), id);
            if (it == ids.end()) return;
            size_t idx = std::distance(ids.begin(), it);
            float h = L * 0.5f;
            std::vector<cv::Point3f> pts = {
                {center.x - h, center.y + h, center.z},
                {center.x + h, center.y + h, center.z},
                {center.x + h, center.y - h, center.z},
                {center.x - h, center.y - h, center.z}
            };
            for (int k = 0; k < 4; ++k) {
                objectPoints.push_back(pts[k]);
                imagePoints.push_back(corners[idx][k]);
            }
        };

        for (const auto &mk : WORLD_MARKERS)
            add_marker(mk.id, mk.pos, MARKER_LENGTH);

        if (objectPoints.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Not enough corner points for PnP (%zu).", objectPoints.size());
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(1);
            return;
        }

        // --- SolvePnPRansac 求解 world→camera_color_optical ---
        cv::Vec3d rvec, tvec;
        std::vector<int> inliers;
        bool ok = cv::solvePnPRansac(
            objectPoints, imagePoints,
            camera_matrix_, dist_coeffs_,
            rvec, tvec, false,
            200, 3.0, 0.99, inliers, cv::SOLVEPNP_EPNP);

        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "PnP failed.");
            return;
        }

        // --- 反轉為 camera_optical→world ---
        // R_cm: world to camera 轉換
        // R_mc: camera 在 world 的位置轉換
        cv::Mat R_cm;
        cv::Rodrigues(rvec, R_cm);
        cv::Mat R_mc = R_cm.t();
        cv::Mat t_mc = -R_mc * cv::Mat(tvec);

        // --- 取 camera_link→camera_color_optical_frame 變換 ---
        tf2::Transform T_clink_copt;
        try {
            auto tf_msg = tf_buffer_->lookupTransform("camera_link", "camera_color_optical_frame", tf2::TimePointZero);
            tf2::fromMsg(tf_msg.transform, T_clink_copt);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }
        tf2::Transform T_copt_clink = T_clink_copt.inverse();

        // --- 把 camera_optical→world 轉成 tf2::Transform ---
        tf2::Matrix3x3 m(
            R_mc.at<double>(0,0), R_mc.at<double>(0,1), R_mc.at<double>(0,2),
            R_mc.at<double>(1,0), R_mc.at<double>(1,1), R_mc.at<double>(1,2),
            R_mc.at<double>(2,0), R_mc.at<double>(2,1), R_mc.at<double>(2,2));
        tf2::Vector3 v(t_mc.at<double>(0), t_mc.at<double>(1), t_mc.at<double>(2));
        tf2::Quaternion q; m.getRotation(q);
        tf2::Transform T_copt_w(q, v);

        // --- world→camera_link = (camera_optical→world)^-1 * (camera_optical→camera_link) ---
        tf2::Transform T_w_clink = T_copt_w * T_copt_clink;

        // --- 廣播 TF ---
        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = "world";
        out.child_frame_id = "camera_link";
        out.transform = tf2::toMsg(T_w_clink);
        tf_broadcaster_->sendTransform(out);

        RCLCPP_INFO(this->get_logger(), "Published world->camera_link (PnP inliers=%zu)", inliers.size());

        cv::imshow("Aruco Detection", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
