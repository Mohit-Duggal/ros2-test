#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode()
        : Node("image_conversion_node"), mode_(2) {  // Default mode is color
        RCLCPP_INFO(this->get_logger(), "Starting ImageConversionNode in Color mode.");

        // Parameters for topic names
        this->declare_parameter<std::string>("input_image_topic", "/usb_cam/image_raw");
        this->declare_parameter<std::string>("output_image_topic", "/processed_image");

        input_topic_ = this->get_parameter("input_image_topic").as_string();
        output_topic_ = this->get_parameter("output_image_topic").as_string();

        // Subscriber to input image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10, std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));

        // Publisher for processed images
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        // Service to change mode
        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "change_mode", std::bind(&ImageConversionNode::changeModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS Image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat processed_image;

            if (mode_ == 1) {
                // Convert to grayscale
                cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_BGR2GRAY);
                cv_ptr->image = processed_image;
                cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
            }

            // Convert back to ROS Image message and publish
            image_pub_->publish(*cv_ptr->toImageMsg());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
        }
    }

    void changeModeCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        mode_ = request->data ? 1 : 2;
        response->success = true;
        response->message = "Mode changed successfully.";
        RCLCPP_INFO(this->get_logger(), "Mode changed to %s.", mode_ == 1 ? "Grayscale" : "Color");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
    std::string input_topic_;
    std::string output_topic_;
    int mode_;  // 1 for grayscale, 2 for color
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
