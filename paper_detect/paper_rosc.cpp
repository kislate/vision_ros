#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <deque>
#include <vector>
#include <cmath>
#include <my_opencv_pkg/square_center.h>
#include <my_opencv_pkg/detector_bool.h>

my_opencv_pkg::detector_bool::ConstPtr detect_flag;  // 全局变量，用于存储检测标志`
// 将四个顶点排序为[左上, 右上, 右下, 左下]
std::vector<cv::Point> order_points(const std::vector<cv::Point>& pts) {
    std::vector<cv::Point> rect(4);
    std::vector<float> s(4);
    for (int i = 0; i < 4; ++i) {
        s[i] = pts[i].x + pts[i].y;
    }
    rect[0] = pts[std::distance(s.begin(), std::min_element(s.begin(), s.end()))];  // 左上
    rect[2] = pts[std::distance(s.begin(), std::max_element(s.begin(), s.end()))];  // 右下

    std::vector<float> diff(4);
    for (int i = 0; i < 4; ++i) {
        diff[i] = pts[i].y - pts[i].x;
    }
    rect[1] = pts[std::distance(diff.begin(), std::min_element(diff.begin(), diff.end()))];  // 右上
    rect[3] = pts[std::distance(diff.begin(), std::max_element(diff.begin(), diff.end()))];  // 左下
    return rect;
}

class PaperTracker {
public:
    PaperTracker(int history_sec = 5) : history(history_sec * 25), stability_threshold(0.75) {}

    cv::Moments extract_features(const std::vector<cv::Point>& cnt) {
        return cv::moments(cnt);
    }

    void update(const std::vector<cv::Point>& cnt) {
        if (!cnt.empty()) {
            history.push_back(cnt);
        }
    }

    std::vector<cv::Point> get_consensus() {
        if (history.size() < 10) {
            return {};
        }

        std::vector<cv::Point2f> centers;
        for (const auto& cnt : history) {
            cv::Moments m = cv::moments(cnt);
            if (m.m00 != 0) {
                centers.push_back(cv::Point2f(m.m10 / m.m00, m.m01 / m.m00));
            }
        }

        if (centers.size() < 10) {
            return {};
        }

        // 计算中心点的均值
        cv::Scalar avg_center_scalar = cv::mean(centers);
        cv::Point2f avg_center(avg_center_scalar[0], avg_center_scalar[1]);

        float min_distance = std::numeric_limits<float>::max();
        std::vector<cv::Point> best_match;
        for (const auto& cnt : history) {
            cv::Moments m = cv::moments(cnt);
            if (m.m00 != 0) {
                cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
                float distance = cv::norm(center - avg_center);
                if (distance < min_distance) {
                    min_distance = distance;
                    best_match = cnt;
                }
            }
        }

        return best_match;
    }

private:
    std::deque<std::vector<cv::Point>> history;
    float stability_threshold;
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg, PaperTracker& tracker, ros::Publisher& center_pub) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        // 预处理
        cv::Mat gray, blurred, edges;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::Canny(blurred, edges, 50, 150);

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 按面积排序，取前三个轮廓
        contours.erase(std::remove_if(contours.begin(), contours.end(), [](const std::vector<cv::Point>& cnt) {
            return cv::contourArea(cnt) < 100;  // 过滤小轮廓
        }), contours.end());
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });
        contours.resize(std::min(contours.size(), size_t(3)));

        // 四边形筛选
        std::vector<cv::Point> paper_contour;
        for (const auto& cnt : contours) {
            double peri = cv::arcLength(cnt, true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(cnt, approx, 0.02 * peri, true);
            if (approx.size() == 4) {
                paper_contour = approx;
                break;
            }
        }

        // 更新跟踪器
        tracker.update(paper_contour);

        // 获取历史共识
        std::vector<cv::Point> consensus = tracker.get_consensus();

        // 可视化标注
        if (!consensus.empty()) {
            // 绘制蓝色轮廓
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{consensus}, -1, cv::Scalar(255, 0, 0), 3);

            // 提取四边形顶点
            std::vector<cv::Point> rect = order_points(consensus);

            // 转换为整数坐标
            for (auto& pt : rect) {
                pt.x = static_cast<int>(pt.x);
                pt.y = static_cast<int>(pt.y);
            }

            // 绘制对角线（黄色）
            cv::line(frame, rect[0], rect[2], cv::Scalar(0, 255, 255), 2);  // 主对角线
            cv::line(frame, rect[1], rect[3], cv::Scalar(0, 255, 255), 2);  // 副对角线

            // 计算对角线交点（中心点）
            int x = (rect[0].x + rect[2].x) / 2;
            int y = (rect[0].y + rect[2].y) / 2;

            // 绘制中心红点
            cv::circle(frame, cv::Point(x, y), 8, cv::Scalar(0, 0, 255), -1);

            // 发布中心点坐标
            my_opencv_pkg::square_center center_msg;
            center_msg.x = x;
            center_msg.y = y;
            center_pub.publish(center_msg);
        }

        if (!paper_contour.empty()) {
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{paper_contour}, -1, cv::Scalar(0, 255, 0), 1);
        }

        // 显示结果
        cv::imshow("Paper Tracking", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
void detector_bool_cb(const my_opencv_pkg::detector_bool::ConstPtr& msg) {
    detect_flag = msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "paper_detect");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    PaperTracker tracker;

    // 创建发布者，发布中心点坐标
    ros::Subscriber detect_sub = nh.subscribe("my_opencv_pkg/detector_bool", 10, detector_bool_cb
    );
    ros::Publisher center_pub = nh.advertise<my_opencv_pkg::square_center>("my_opencv_pkg/square_center", 10);

    while (ros::ok())
    {
        ros::spinOnce();  // 处理ROS消息
        // 等待订阅者连接
        std::cout << "Waiting for subscribers..." << std::endl;
        std::cout << "detect_flag: " << detect_flag->detect << std::endl;
        if (detect_flag->detect)
        {
            break;  // 如果检测到目标，跳出循环
        }
        ros::Duration(0.1).sleep();  // 等待一段时间
    }
    
    // 订阅图像话题，并将发布者作为参数传递给回调函数
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, boost::bind(imageCallback, _1, boost::ref(tracker), boost::ref(center_pub)));

    ros::spin();
    return 0;
}