#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ImageConversionNode : public rclcpp::Node
{
public:
    ImageConversionNode() : Node("image_conversion_node"), mode_(1)
    {
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/usb_cam/image_raw", 10,
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));

        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

        
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_mode",
            std::bind(&ImageConversionNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Image Conversion Node has started.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::Mat output_frame;
            if (mode_ == 1)
            {

              
                cv::cvtColor(frame, output_frame, cv::COLOR_BGR2GRAY);
                
                auto output_msg = cv_bridge::CvImage(msg->header, "mono8", output_frame).toImageMsg();
                publisher_->publish(*output_msg);
            }
            else
            {
                
                publisher_->publish(*msg);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to process frame: %s", e.what());
        }
    }

    void setModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            mode_ = 1;
            RCLCPP_INFO(this->get_logger(), "Switched to Grayscale mode.");
        }
        else
        {
            mode_ = 2;
            RCLCPP_INFO(this->get_logger(), "Switched to Color mode.");
        }
        response->success = true;
        response->message = "Mode updated successfully.";
    }

    int mode_; 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}