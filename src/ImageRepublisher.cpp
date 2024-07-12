#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ImageRepublisher : public rclcpp::Node
{
public:
    ImageRepublisher()
        : Node("image_transport_publisher")
    {
        image_transport::ImageTransport it(shared_from_this());
        std::cout << "0" << std::endl;
        image_pub_ = it.advertise("camera/image", 1);
        std::cout << "1" << std::endl;
        image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("/thermal_image/compressed", 10,std::bind(&ImageRepublisher::image_callback,this,std::placeholders::_1));
        std::cout << "2" << std::endl;
        msg.header.stamp = this->now();
        std::cout << "3" << std::endl;
        msg.header.frame_id = "camera_frame";
        std::cout << "4" << std::endl;
    }

private:
    void image_callback (const sensor_msgs::msg::CompressedImage::SharedPtr compressedImage){
        std::vector<uchar> data(compressedImage->data.begin(),compressedImage->data.end());
        cv::Mat image = cv::imdecode(data,cv::IMREAD_COLOR);
        
        if (image.empty()) {
            std::cout << "Failed to decode image" << std::endl;
            return;
        }


        msg.height = image.rows;
        msg.width = image.cols;
        msg.encoding = "bgr8";
        msg.is_bigendian = 0;
        msg.step = image.cols * 3;
        msg.data.assign(image.data, image.data + image.total() * image.elemSize());


        image_pub_.publish(msg);
    }
    sensor_msgs::msg::Image msg;
    image_transport::Publisher image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
};

int main(int argc, char **argv)
{
    std::cout << "started" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "init" << std::endl;
    rclcpp::spin(std::make_shared<ImageRepublisher>());
    std::cout << "spin" << std::endl;
    rclcpp::shutdown();
    return 0;
}