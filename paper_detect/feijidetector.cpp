#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

// 全局变量，用于存储滑动条的值
int white_threshold = 200; // RGB 值的阈值，高于此值认为是白色

// 回调函数，用于更新滑动条的值
void onTrackbarThreshold(int pos, void* userdata) {
    white_threshold = pos;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;

        // 创建一个与原图像大小相同的空白图像，用于存储二值化结果
        cv::Mat binary = cv::Mat::zeros(img.size(), CV_8UC3);

        // 遍历每个像素
        for (int y = 0; y < img.rows; ++y)
        {
            for (int x = 0; x < img.cols; ++x)
            {
                cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
                int r = pixel[2];
                int g = pixel[1];
                int b = pixel[0];

                // 检查 RGB 值是否都高于阈值
                if (r > white_threshold && g > white_threshold && b > white_threshold)
                {
                    binary.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // 设置为白色
                }
                else
                {
                    binary.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // 设置为黑色
                }
            }
        }

        // 显示结果
        cv::imshow("USB Camera", binary);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgb_binary_processor");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    cv::namedWindow("USB Camera", cv::WINDOW_AUTOSIZE);

    // 创建滑动条
    cv::createTrackbar("White Threshold", "USB Camera", &white_threshold, 255, onTrackbarThreshold);

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}