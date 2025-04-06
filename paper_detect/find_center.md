以下是一个基于ROS C++的代码示例，用于订阅USB摄像头的图像话题`/usb_cam/image_raw`，计算画面中心点的像素坐标，并在图像中心点绘制一个红色圆点。代码使用了`cv_bridge`库将ROS图像消息转换为OpenCV格式，以便进行图像处理和显示。

### 示例代码
```cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 回调函数
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

    // 开始处理ROS消息
    ros::spin();

    return 0;
}
```

### 代码说明
1. **依赖库**：
   - `cv_bridge`：用于将ROS图像消息转换为OpenCV格式。
   - `sensor_msgs`：用于处理图像消息。
   - `opencv2`：用于图像处理和显示。

2. **回调函数**：
   - `imageCallback`函数接收图像消息，使用`cv_bridge`将其转换为OpenCV图像。
   - 计算图像的宽度和高度，进而得到中心点的像素坐标。
   - 使用`cv::circle`在图像中心点绘制一个红色圆点。
   - 使用`cv::imshow`显示处理后的图像。

3. **主函数**：
   - 初始化ROS节点`image_center_point`。
   - 订阅`/usb_cam/image_raw`话题。
   - 初始化OpenCV窗口以显示图像。
   - 使用`ros::spin`处理ROS消息。

### 编译
在`CMakeLists.txt`文件中添加以下内容：
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_center_point src/image_center_point.cpp)

target_link_libraries(image_center_point
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

### 注意事项
- 确保安装了`cv_bridge`、`sensor_msgs`和`opencv`。
- 如果摄像头话题名称不是`/usb_cam/image_raw`，请修改代码中的订阅话题名称。
- 如果需要在ROS中发布处理后的图像，可以使用`image_transport`库将处理后的图像发布到新的话题。

运行此代码后，你将看到一个窗口显示摄像头图像，并且图像中心点被一个红色圆点标注。