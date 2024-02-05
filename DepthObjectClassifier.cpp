#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include "yolov7_ros/PointCenterArray.h"

// 전역 변수
cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub;

const double fx = 524.4417;
const double fy = 524.2536;
const double cx = 299.4899;
const double cy = 305.6905;

void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void pointCenterCallback(const yolov7_ros::PointCenterArray::ConstPtr& msg) {
    if (!cv_ptr) return;

    for (const auto& point : msg->points) {
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);

        if (x >= 0 && x < cv_ptr->image.cols && y >= 0 && y < cv_ptr->image.rows) {
            float d = cv_ptr->image.at<ushort>(y, x); // Assuming depth image is UINT16
            float X = (x - cx) * d / fx;
            float Y = -(y - cy) * d / fy;
            float Z = d;

            ROS_INFO("PointCenter: X=%f, Y=%f, Z=%f", X, Y, Z);

            std_msgs::String msg;
            if (-80 <= X && X <= -50 && -120 <= Y && Y <= -80 && Z <= 1000) {
                msg.data = "Action A";
            } else if (60 <= X && X <= 120 && -160 <= Y && Y <= -90 && Z <= 1000) {
                msg.data = "Action B";
            } else if (190 <= X && X <= 250 && -100 <= Y && Y <= -70 && Z <= 1000) {
                msg.data = "Action C";
            } else {
                continue; // No action required for this point
            }
            pub.publish(msg);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_pointcenter_listener");
    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::String>("/keyboard_action_result", 10);

    ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_raw", 1, depthCallback);
    ros::Subscriber pointcenter_sub = nh.subscribe("/yolov7/pointcenter_topic", 1, pointCenterCallback);

    ros::spin();
    return 0;
}

