#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ImageRepublisher : public rclcpp::Node
{
public:
    ImageRepublisher()
        : Node("n10_image_transport_publisher")
    {}
    void init(image_transport::ImageTransport &it){
        image_pub_ = it.advertise("/n10/thermal/color", 1);
        msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_frame";
    }
    void setup(){
        
        image_sub = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/thermal_image/compressed", 10,std::bind(&ImageRepublisher::image_callback,this,std::placeholders::_1));
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageRepublisher>();
    image_transport::ImageTransport it(node);
    node->init(it);
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}