#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include "yolov7_ros/PointCenterArray.h"
#include <thread>
#include <chrono>
#include <map>

class IntegratedNode {
public:
    IntegratedNode() {
        // Initialize ROS publisher and subscribers
        pub_ = nh_.advertise<std_msgs::String>("/keyboard_action_result", 10);
        depth_sub_ = nh_.subscribe("/camera/depth/image_raw", 1, &IntegratedNode::depthCallback, this);
        pointcenter_sub_ = nh_.subscribe("/yolov7/pointcenter_topic", 1, &IntegratedNode::pointCenterCallback, this);
        ui_sub_ = nh_.subscribe("/UI_result", 10, &IntegratedNode::uiCallback, this);
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void pointCenterCallback(const yolov7_ros::PointCenterArray::ConstPtr& msg) {
        if (!cv_ptr_) return;

        for (const auto& point : msg->points) {
            int x = static_cast<int>(point.x);
            int y = static_cast<int>(point.y);

            if (x >= 0 && x < cv_ptr_->image.cols && y >= 0 && y < cv_ptr_->image.rows) {
                float d = cv_ptr_->image.at<ushort>(y, x); // Assuming depth image is UINT16
                float X = (x - cx) * d / fx;
                float Y = -(y - cy) * d / fy;
                float Z = d;

                ROS_INFO("PointCenter: X=%f, Y=%f, Z=%f", X, Y, Z);

                std_msgs::String action_msg;
                if (-80 <= X && X <= -50 && -120 <= Y && Y <= -80 && Z <= 1000) {
                    action_msg.data = "Action A";
                } else if (60 <= X && X <= 120 && -160 <= Y && Y <= -90 && Z <= 1000) {
                    action_msg.data = "Action B";
                } else if (190 <= X && X <= 250 && -100 <= Y && Y <= -70 && Z <= 1000) {
                    action_msg.data = "Action C";
                } else {
                    continue; // No action required for this point
                }
                pub_.publish(action_msg);
            }
        }
    }

    void uiCallback(const std_msgs::String::ConstPtr& msg) {
        std::string action;
        if (msg->data == "a") {
            action = "i, g, d, s, f, d, i, o, g, i";
        } else if (msg->data == "b") {
            action = "i, g, a, f, i, o, g, i";
        } else if (msg->data == "c") {
            action = "i, g, x, z, f, x, i, o , g, i";
        }

        if (!action.empty()) {
            std::thread([this, action]() {
                auto actions = splitActions(action);
                for (const auto& act : actions) {
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    std_msgs::String msg;
                    msg.data = act;
                    pub_.publish(msg);
                }
            }).detach();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber depth_sub_, pointcenter_sub_, ui_sub_;
    cv_bridge::CvImagePtr cv_ptr_;

   

