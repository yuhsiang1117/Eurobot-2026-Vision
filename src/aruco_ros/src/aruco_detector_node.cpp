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

const double MARKER_LENGTH = 0.1; // meter

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

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_result_image", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.1;
        detector_params_->adaptiveThreshWinSizeMin = 7;
        detector_params_->adaptiveThreshWinSizeMax = 47;
        detector_params_->adaptiveThreshWinSizeStep = 10;
        detector_params_->minCornerDistanceRate = 0.05;
        detector_params_->minMarkerDistanceRate = 0.05;

        camera_matrix_ = (cv::Mat1d(3, 3) <<
        916.026611328125, 0.0,       653.2020263671875,
        0.0,       913.7075805664062, 366.0958251953125,
        0.0,       0.0,       1.0);
        dist_coeffs_ = (cv::Mat1d(1, 5) << 0.124547, -0.188246, -0.006140, 0.001525, 0.000000); //0.124547, -0.188246, -0.006140, 0.001525, 0.000000

        rvec_ = cv::Mat::zeros(3, 1, CV_64F);
        tvec_ = cv::Mat::zeros(3, 1, CV_64F);
        filtered_rvec_ = cv::Mat::zeros(3, 1, CV_64F);
        filtered_tvec_ = cv::Mat::zeros(3, 1, CV_64F);
        have_prev_pose_ = false;

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
        cv::Mat blurred_frame;
        cv::GaussianBlur(frame, blurred_frame, cv::Size(5, 5), 0);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(blurred_frame, dictionary_, corners, ids, detector_params_, rejected);
        if (ids.empty()) {
            publish_debug_image(frame, msg->header); // 發布原圖
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
            publish_debug_image(frame, msg->header);
            return;
        }

        int pnp_method = cv::SOLVEPNP_ITERATIVE; // 預設：多標記用迭代法

        if (objectPoints.size() == 4) {
            pnp_method = cv::SOLVEPNP_IPPE_SQUARE;
        }

        // --- SolvePnPRansac 求解 world→camera_color_optical ---
        cv::Mat curr_rvec = rvec_.clone();
        cv::Mat curr_tvec = tvec_.clone();
        bool use_guess = have_prev_pose_;
        std::vector<int> inliers;

        bool ok = cv::solvePnP(
            objectPoints, imagePoints,
            camera_matrix_, dist_coeffs_,
            curr_rvec, curr_tvec, use_guess,
            pnp_method);

        if (!ok ) {//|| inliers.size() < 4
            RCLCPP_WARN(this->get_logger(), "PnP failed.");
            publish_debug_image(frame, msg->header);
            return;
        }

        if (!have_prev_pose_) {
            filtered_rvec_ = curr_rvec.clone();
            filtered_tvec_ = curr_tvec.clone();
        } else {
            // alpha: 0.2~0.3 會非常平滑，類似 pytagmapper 的效果
            double alpha = 0.2; 
            filtered_tvec_ = alpha * curr_tvec + (1.0 - alpha) * filtered_tvec_;
            filtered_rvec_ = alpha * curr_rvec + (1.0 - alpha) * filtered_rvec_;
        }
        rvec_ = filtered_rvec_.clone();
        tvec_ = filtered_tvec_.clone();
        have_prev_pose_ = true;

        for (const auto& pt : imagePoints) {
            cv::circle(frame, pt, 4, cv::Scalar(0, 255, 0), -1); 
        }
        std::string info = "Optimization Points: " + std::to_string(objectPoints.size());
        cv::putText(frame, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // --- 反轉為 camera_optical→world ---
        cv::Mat R_cm;
        cv::Rodrigues(rvec_, R_cm);
        cv::Mat R_mc = R_cm.t();
        cv::Mat t_mc = -R_mc * cv::Mat(tvec_);

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

        RCLCPP_INFO(this->get_logger(), "Published (Negotiated). Points: %zu", objectPoints.size());

        publish_debug_image(frame, msg->header);
    }

    void publish_debug_image(const cv::Mat& img, const std_msgs::msg::Header& header)
    {
        // 轉換 cv::Mat -> sensor_msgs::msg::Image
        sensor_msgs::msg::Image::SharedPtr out_msg;
        try {
            // 使用原始 msg 的 header，保持時間戳同步
            out_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
            image_pub_->publish(*out_msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    cv::Mat rvec_, tvec_;
    cv::Mat filtered_rvec_, filtered_tvec_;
    bool have_prev_pose_;
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
