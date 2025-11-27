#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/stat.h>

class CameraCalibratorNode : public rclcpp::Node
{
public:
    CameraCalibratorNode() : Node("camera_calibrator_node")
    {
        // --- Parameters ---
        this->declare_parameter<int>("squares_x", 7);
        this->declare_parameter<int>("squares_y", 5);
        this->declare_parameter<float>("square_length", 0.04f);
        this->declare_parameter<float>("marker_length", 0.02f);
        this->declare_parameter<std::string>("dictionary", "DICT_4X4_100");
        this->declare_parameter<std::string>("output_file", "camera_calibration.yaml");
        this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");

        int squares_x = this->get_parameter("squares_x").as_int();
        int squares_y = this.get_parameter("squares_y").as_int();
        float square_length = this->get_parameter("square_length").as_double();
        float marker_length = this->get_parameter("marker_length").as_double();
        std::string dictionary_name = this->get_parameter("dictionary").as_string();
        output_file_ = this->get_parameter("output_file").as_string();
        std::string image_topic = this->get_parameter("image_topic").as_string();

        // --- ArUco and Charuco Setup ---
        int dict_id = cv::aruco::DICT_4X4_100; // Default
        if (dictionary_name == "DICT_5X5_100") dict_id = cv::aruco::DICT_5X5_100;
        // Add more dictionary options if needed

        dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
        board_ = cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(squares_x, squares_y), square_length, marker_length, dictionary_);
        detector_params_ = cv::aruco::DetectorParameters::create();

        // --- ROS2 Setup ---
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&CameraCalibratorNode::image_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Camera calibrator node started.");
        RCLCPP_INFO(this->get_logger(), "Listening for images on topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Press 'c' to capture a frame. Press 's' to calibrate and save.");
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
        cv::Mat frame_copy;
        frame.copyTo(frame_copy);

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, detector_params_);

        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame_copy, marker_corners, marker_ids);
            std::vector<cv::Point2f> charuco_corners;
            std::vector<int> charuco_ids;
            cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, frame, board_, charuco_corners, charuco_ids);

            if (!charuco_ids.empty()) {
                cv::aruco::drawDetectedCornersCharuco(frame_copy, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
            }
        }

        cv::imshow("Calibration View", frame_copy);
        char key = (char)cv::waitKey(1);

        if (key == 'c') {
            capture_frame(frame);
        } else if (key == 's') {
            calibrate_and_save();
        }
    }

    void capture_frame(const cv::Mat& frame)
    {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, detector_params_);

        if (marker_ids.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Not enough markers detected to capture frame.");
            return;
        }

        std::vector<cv::Point2f> charuco_corners;
        std::vector<int> charuco_ids;
        cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, frame, board_, charuco_corners, charuco_ids);

        if (charuco_ids.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Not enough Charuco corners interpolated.");
            return;
        }

        all_charuco_corners_.push_back(charuco_corners);
        all_charuco_ids_.push_back(charuco_ids);
        image_size_ = frame.size();

        RCLCPP_INFO(this->get_logger(), "Frame captured. Total frames: %zu", all_charuco_corners_.size());
    }

    void calibrate_and_save()
    {
        if (all_charuco_corners_.size() < 5) {
            RCLCPP_ERROR(this->get_logger(), "Not enough frames for calibration. At least 5 are required.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting calibration...");

        cv::Mat camera_matrix, dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        int flags = cv::CALIB_FIX_ASPECT_RATIO;

        double rms = cv::aruco::calibrateCameraCharuco(
            all_charuco_corners_, all_charuco_ids_, board_,
            image_size_, camera_matrix, dist_coeffs, rvecs, tvecs, flags);

        RCLCPP_INFO(this->get_logger(), "Calibration finished. RMS error: %f", rms);
        RCLCPP_INFO(this->get_logger(), "Camera Matrix:\n%s", cv::format(camera_matrix, cv::Formatter::FMT_C).c_str());
        RCLCPP_INFO(this->get_logger(), "Distortion Coefficients:\n%s", cv::format(dist_coeffs, cv::Formatter::FMT_C).c_str());

        // Save to file
        std::string package_path = ament_index_cpp::get_package_share_directory("aruco_ros");
        std::string config_dir = package_path + "/config";
        
        // Create config directory if it doesn't exist
        struct stat info;
        if (stat(config_dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            if (mkdir(config_dir.c_str(), 0755) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create config directory: %s", config_dir.c_str());
                return;
            }
        }
        
        std::string file_path = config_dir + "/" + output_file_;

        cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file for writing: %s", file_path.c_str());
            return;
        }

        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs.release();

        RCLCPP_INFO(this->get_logger(), "Calibration data saved to: %s", file_path.c_str());
        
        // Shutdown after saving
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> board_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    std::string output_file_;
    cv::Size image_size_;

    std::vector<std::vector<std::vector<cv::Point2f>>> all_marker_corners_;
    std::vector<std::vector<int>> all_marker_ids_;
    std::vector<std::vector<cv::Point2f>> all_charuco_corners_;
    std::vector<std::vector<int>> all_charuco_ids_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCalibratorNode>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    return 0;
}
