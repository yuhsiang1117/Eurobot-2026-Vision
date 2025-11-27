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
#include <mutex>
#include <cmath>

const double MARKER_LENGTH = 0.1;   //meter

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
        // ROS parameters (可在 launch 或 ros2 param set 調整)
        this->declare_parameter("alpha_translation", 0.2);
        this->declare_parameter("alpha_rotation", 0.2);
        this->get_parameter("alpha_translation", alpha_trans_);
        this->get_parameter("alpha_rotation", alpha_rot_);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ArucoDetectorNode::image_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        detector_params_ = cv::aruco::DetectorParameters::create();

        // 偵測參數
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        detector_params_->cornerRefinementWinSize = 9;
        detector_params_->cornerRefinementMaxIterations = 90;
        detector_params_->cornerRefinementMinAccuracy = 0.03;

        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 45;
        detector_params_->adaptiveThreshWinSizeStep = 4;
        detector_params_->adaptiveThreshConstant = 7;

        detector_params_->minMarkerPerimeterRate = 0.03;
        detector_params_->maxMarkerPerimeterRate = 4.0;
        detector_params_->minCornerDistanceRate = 0.03;
        detector_params_->minMarkerDistanceRate = 0.05;

        // 相機內參
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            916.026611328125, 0.0, 653.2020263671875,
            0.0, 913.7075805664062, 366.0958251953125,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        has_prev_ = false;
    }
private:
    tf2::Quaternion slerp_quat(const tf2::Quaternion &q1_in, const tf2::Quaternion &q2_in, double t) {
        // copy because we may flip sign on q2
        tf2::Quaternion q1 = q1_in;
        tf2::Quaternion q2 = q2_in;

        double dot = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() + q1.w()*q2.w();
        // if dot < 0, negate q2 to take shorter path
        if (dot < 0.0) {
            q2 = tf2::Quaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
            dot = -dot;
        }
        const double DOT_THRESH = 0.9995;
        if (dot > DOT_THRESH) {
            // very close - use linear interpolation then normalize
            tf2::Quaternion res(
                q1.x() + t*(q2.x() - q1.x()),
                q1.y() + t*(q2.y() - q1.y()),
                q1.z() + t*(q2.z() - q1.z()),
                q1.w() + t*(q2.w() - q1.w())
            );
            res.normalize();
            return res;
        }
        // theta_0 = angle between input quaternions
        double theta_0 = std::acos(dot);
        double sin_theta_0 = std::sin(theta_0);
        double s0 = std::sin((1.0 - t) * theta_0) / sin_theta_0;
        double s1 = std::sin(t * theta_0) / sin_theta_0;
        tf2::Quaternion res(
            s0 * q1.x() + s1 * q2.x(),
            s0 * q1.y() + s1 * q2.y(),
            s0 * q1.z() + s1 * q2.z(),
            s0 * q1.w() + s1 * q2.w()
        );
        res.normalize();
        return res;
    }

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
        cv::Mat gray;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        // 在dectect Aruco前先做灰階處理
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0);
        clahe->apply(gray, gray);
        cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);
        if (ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "no aruco");
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(1);
            return;
        }

        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        // RCLCPP_INFO(this->get_logger(), "Detected %zu markers.", ids.size());

        // 建立 solvePnP 所需的點對
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
            RCLCPP_INFO(this->get_logger(), "Not enough corner points for PnP (%zu).", objectPoints.size());
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(1);
            return;
        }

        // SolvePnP求解 world → camera_color_optical_frame
        cv::Vec3d rvec, tvec;
        bool ok = cv::solvePnP(
            objectPoints, imagePoints,
            camera_matrix_, dist_coeffs_,
            rvec, tvec, false,
            cv::SOLVEPNP_ITERATIVE
        );

        // 若 PnP 失敗，直接使用上一幀的 TF 避免跳動
        if (!ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "PnP failed, using previous transform.");

            std::lock_guard<std::mutex> lk(prev_mutex_);
            if (has_prev_) {
                geometry_msgs::msg::TransformStamped out;
                out.header.stamp = this->get_clock()->now();
                out.header.frame_id = "world";
                out.child_frame_id = "camera_link";
                out.transform = tf2::toMsg(tf2::Transform(prev_q_, prev_t_));
                tf_broadcaster_->sendTransform(out);
            }
            return;
        }

        // Rodrigues 求得旋轉矩陣(camera_optical → world)
        cv::Mat R_cm;
        cv::Rodrigues(rvec, R_cm);
        cv::Mat R_mc = R_cm.t();
        cv::Mat t_mc = -R_mc * cv::Mat(tvec);

        // 取得 TF: camera_link → camera_color_optical_frame
        tf2::Transform T_clink_copt;

        try {
            auto tf_msg = tf_buffer_->lookupTransform(
                "camera_link", "camera_color_optical_frame",
                tf2::TimePointZero
            );
            tf2::fromMsg(tf_msg.transform, T_clink_copt);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),
                "TF lookup failed: %s, using previous transform.", ex.what());

            // 若第一幀就失敗 → 無法 fallback
            std::lock_guard<std::mutex> lk(prev_mutex_);
            if (!has_prev_) return;

            // 回傳前一幀
            geometry_msgs::msg::TransformStamped out;
            out.header.stamp = this->get_clock()->now();
            out.header.frame_id = "world";
            out.child_frame_id = "camera_link";
            out.transform = tf2::toMsg(tf2::Transform(prev_q_, prev_t_));
            tf_broadcaster_->sendTransform(out);
            return;
        }

        // camera_optical → world
        tf2::Matrix3x3 m(
            R_mc.at<double>(0,0), R_mc.at<double>(0,1), R_mc.at<double>(0,2),
            R_mc.at<double>(1,0), R_mc.at<double>(1,1), R_mc.at<double>(1,2),
            R_mc.at<double>(2,0), R_mc.at<double>(2,1), R_mc.at<double>(2,2)
        );
        tf2::Vector3 v(
            t_mc.at<double>(0),
            t_mc.at<double>(1),
            t_mc.at<double>(2)
        );
        tf2::Quaternion q;
        m.getRotation(q);

        tf2::Transform T_copt_w(q, v);

        // camera_optical → camera_link = (camera_link → camera_optical)^(-1)
        tf2::Transform T_copt_clink = T_clink_copt.inverse();

        // world → camera_link = (camera_optical → world) * (camera_optical → camera_link)
        tf2::Transform T_w_clink = T_copt_w * T_copt_clink;

        // 5. 平滑（EMA 平移 + SLERP 旋轉）
        std::lock_guard<std::mutex> lk(prev_mutex_);

        tf2::Vector3 cur_t = T_w_clink.getOrigin();
        tf2::Quaternion cur_q = T_w_clink.getRotation();

        if (!has_prev_) {
            // 第一幀直接初始化平滑變數
            prev_t_ = cur_t;
            prev_q_ = cur_q;
            has_prev_ = true;
        }
        else {
            // EMA 平滑平移
            prev_t_ = tf2::Vector3(
                alpha_trans_ * cur_t.x() + (1 - alpha_trans_) * prev_t_.x(),
                alpha_trans_ * cur_t.y() + (1 - alpha_trans_) * prev_t_.y(),
                alpha_trans_ * cur_t.z() + (1 - alpha_trans_) * prev_t_.z()
            );

            // SLERP 平滑旋轉
            prev_q_ = slerp_quat(prev_q_, cur_q, alpha_rot_);
            prev_q_.normalize();
        }

        // 6. 發布 TF: world → camera_link（平滑後的）
        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = "world";
        out.child_frame_id = "camera_link";
        out.transform = tf2::toMsg(tf2::Transform(prev_q_, prev_t_));
        tf_broadcaster_->sendTransform(out);

        // Debug RPY 輸出
        tf2::Matrix3x3 mm(prev_q_);
        double roll, pitch, yaw;
        mm.getRPY(roll, pitch, yaw);

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "Published world->camera_link. RPY=%.2f %.2f %.2f deg, t=(%.3f %.3f %.3f m)",
        //     roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI,
        //     prev_t_.x(), prev_t_.y(), prev_t_.z()
        // );

        cv::imshow("Aruco Detection", frame);
        cv::waitKey(1);
    }

    // members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    // smoothing state
    std::mutex prev_mutex_;
    tf2::Vector3 prev_t_;
    tf2::Quaternion prev_q_;
    bool has_prev_;

    double alpha_trans_ = 0.2;
    double alpha_rot_ = 0.2;
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