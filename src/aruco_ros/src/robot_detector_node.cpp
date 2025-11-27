#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class RobotArucoNode : public rclcpp::Node {
public:
    RobotArucoNode() : Node("robot_aruco_detector") {
        using std::placeholders::_1;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&RobotArucoNode::image_callback, this, _1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

        // 相機內參
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            916.026611328125, 0.0, 653.2020263671875,
            0.0, 913.7075805664062, 366.0958251953125,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1,5,CV_64F);
        marker_length_ = 0.1;  // ArUco 尺寸（公尺）

        // 場上兩台機器頭頂 ID
        target_robot_ids_ = {2, 6};
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image;

        // Step 1: Aruco detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids);
        if (ids.empty()) {
            return;
        }

        // Step 2: iterate robot IDs
        for (size_t i = 0; i < ids.size(); i++) {
            int id = ids[i];
            bool matched = (std::find(
                target_robot_ids_.begin(),
                target_robot_ids_.end(),
                id
            ) != target_robot_ids_.end());
            if (!matched) {
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "MATCH robot ID: %d", id);

            // ==== Step 3: 建立 Aruco 3D corner points（以 marker 中心為原點）====
            float L = marker_length_; // 你的機器人 marker 長度 (meters)
            float h = L * 0.5f;
            float height = 0.0f;
            std::vector<cv::Point3f> objPoints = {
                {-h,  h, height},
                { h,  h, height},
                { h, -h, height},
                {-h, -h, height}
            };
            std::vector<cv::Point2f> imgPoints = corners[i];

            // ==== Step 4: solvePnPRansac ====
            cv::Vec3d rvec, tvec;
            bool ok = cv::solvePnP(
                objPoints, imgPoints,
                camera_matrix_, dist_coeffs_,
                rvec, tvec, false,
                cv::SOLVEPNP_ITERATIVE
            );
            if (!ok) {
                RCLCPP_WARN(this->get_logger(), "PnP failed for robot ID %d", id);
                continue;
            }

            // ==== Step 5: rvec / tvec → Transform (camera_optical → robot) ====
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            tf2::Matrix3x3 rot(
                R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
            );
            tf2::Quaternion q;
            rot.getRotation(q);

            tf2::Transform T_copt_robot(q, tf2::Vector3(tvec[0], tvec[1], tvec[2]));

            // ==== Step 6: lookup world → camera_optical ====
            tf2::Transform T_w_copt;
            try {
                auto tf_msg = tf_buffer_->lookupTransform(
                    "world", "camera_color_optical_frame", tf2::TimePointZero
                );
                tf2::fromMsg(tf_msg.transform, T_w_copt);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                continue;
            }

            // ==== Step 7: world → robot ====
            tf2::Transform T_w_robot = T_w_copt * T_copt_robot;

            // ==== Step 8: publish TF ====
            geometry_msgs::msg::TransformStamped out;
            out.header.stamp = this->get_clock()->now();
            out.header.frame_id = "world";
            out.child_frame_id = "robot_" + std::to_string(id);

            tf2::convert(T_w_robot, out.transform);

            tf_broadcaster_->sendTransform(out);
        }
    }
    // void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    //     cv_bridge::CvImagePtr cv_ptr;
    //     try {
    //         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //     } catch (cv_bridge::Exception &e) {
    //         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    //         return;
    //     }
    //     cv::Mat frame = cv_ptr->image;

    //     std::vector<int> ids;
    //     std::vector<std::vector<cv::Point2f>> corners;
    //     cv::aruco::detectMarkers(frame, dictionary_, corners, ids);

    //     if (ids.empty()) {
    //         return;
    //     }

    //     for (size_t i = 0; i < ids.size(); i++) {
    //         int id = ids[i];
    //         bool matched = std::find(
    //             target_robot_ids_.begin(),
    //             target_robot_ids_.end(),
    //             id
    //         ) != target_robot_ids_.end();
    //         if (!matched) {
    //             RCLCPP_INFO(this->get_logger(), "no match robot");
    //             continue;
    //         }
    //         RCLCPP_INFO(this->get_logger(), "MATCHED robot ID: %d", id);

    //         // === 修正：estimatePoseSingleMarkers 必須用 vector<Vec3d> ===
    //         std::vector<cv::Vec3d> rvecs, tvecs;
    //         cv::aruco::estimatePoseSingleMarkers(
    //             std::vector<std::vector<cv::Point2f>>{corners[i]},
    //             marker_length_,
    //             camera_matrix_,
    //             dist_coeffs_,
    //             rvecs,
    //             tvecs
    //         );

    //         cv::Vec3d rvec = rvecs[0];
    //         cv::Vec3d tvec = tvecs[0];

    //         // Convert to tf2
    //         cv::Mat R;
    //         cv::Rodrigues(rvec, R);

    //         tf2::Matrix3x3 rot(
    //             R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
    //             R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
    //             R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    //         );

    //         tf2::Quaternion q;
    //         rot.getRotation(q);

    //         tf2::Transform T_copt_robot(q, tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    //         // === lookup world->camera_optical ===
    //         tf2::Transform T_w_copt;
    //         try {
    //             auto tf_msg = tf_buffer_->lookupTransform(
    //                 "world", "camera_color_optical_frame", tf2::TimePointZero
    //             );
    //             tf2::fromMsg(tf_msg.transform, T_w_copt);
    //         } catch (tf2::TransformException &ex) {
    //             RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    //             continue;
    //         }

    //         // World to robot
    //         tf2::Transform T_w_robot = T_w_copt * T_copt_robot;

    //         geometry_msgs::msg::TransformStamped out;
    //         out.header.stamp = this->get_clock()->now();
    //         out.header.frame_id = "world";
    //         out.child_frame_id = "robot_" + std::to_string(id);

    //         // === tf2::convert 需要 include <tf2_geometry_msgs> ===
    //         tf2::convert(T_w_robot, out.transform);

    //         tf_broadcaster_->sendTransform(out);
    //     }
    // }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat camera_matrix_, dist_coeffs_;
    double marker_length_;
    std::vector<int> target_robot_ids_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotArucoNode>());
    rclcpp::shutdown();
    return 0;
}