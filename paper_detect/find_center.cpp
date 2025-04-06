#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 全局变量，用于存储点击点的坐标
cv::Point click_point(-1, -1);

// 鼠标回调函数
void mouseCallback(int event, int x, int y, int flags, void* param)
{
    if (event == cv::EVENT_LBUTTONDOWN) // 左键点击事件
    {
        click_point = cv::Point(x, y);
        ROS_INFO("Clicked point: (%d, %d)", click_point.x, click_point.y);
    }
}

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 使用cv_bridge将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 获取图像的宽度和高度
        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;

        // 计算图像中心点的像素坐标
        int center_x = width / 2;
        int center_y = height / 2;

        // 在图像中心点绘制一个红色圆点
        cv::circle(cv_ptr->image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);

        // 如果有点击点，绘制点击点
        if (click_point.x >= 0 && click_point.y >= 0)
        {
            cv::circle(cv_ptr->image, click_point, 5, cv::Scalar(0, 255, 0), -1); // 绿色圆点
        }

        // 显示图像
        cv::imshow("Image with Center Point", cv_ptr->image);

        // 等待键盘事件
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "image_center_point");
    ros::NodeHandle nh;

    // 订阅USB摄像头的图像话题
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // 初始化OpenCV窗口
    cv::namedWindow("Image with Center Point", cv::WINDOW_AUTOSIZE);

    // 设置鼠标回调函数
    cv::setMouseCallback("Image with Center Point", mouseCallback, NULL);

    // 开始处理ROS消息
    ros::spin();

    return 0;
}