#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageSubscriber {
public:
    ImageSubscriber() {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 订阅图像消息
        image_sub_ = nh.subscribe("/image_topic", 1, &ImageSubscriber::imageCallback, this);

        // 创建OpenCV窗口
        cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    }

    ~ImageSubscriber() {
        // 销毁OpenCV窗口
        cv::destroyWindow("Image");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 在窗口中显示图像
            cv::imshow("Image", cv_ptr->image);

            // 等待用户按下键盘上的任意键
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    ros::Subscriber image_sub_;
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "image_subscriber");

    // 创建图像订阅对象
    ImageSubscriber image_subscriber;

    // 循环等待ROS消息
    ros::spin();

    return 0;
}
