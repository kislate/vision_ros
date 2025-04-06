# 编译
## 在CMakeLists.txt中添加以下内容：
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(paper_tracker src/paper_tracker.cpp)
target_link_libraries(paper_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```
## 在package.xml中添加以下依赖：
```xml
<build_depend>cv_bridge</build_depend>
<build_depend>image_transport</build_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>image_transport</exec_depend>
```
运行catkin_make进行编译。
# 运行
## 运行节点：
```bash
roscore
rosrun paper_tracker paper_tracker
```
确保你的USB摄像头已正确连接，并且/usb_cam/image_raw话题正在发布图像数据。