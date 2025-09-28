#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// DDS includes
#include "WheelEncoderMsg.h"
#include "WheelEncoderMsgSubscriber.h"

using namespace std::chrono_literals;

class EncoderOdomNode : public rclcpp::Node
{
public:
    EncoderOdomNode()
    : Node("encoder_odom_node"),
      x_(0.0), y_(0.0), theta_(0.0)
    {
        // Thông số robot
        declare_parameter("wheel_radius", 0.0325);   // 6.5 / 2
        declare_parameter("wheel_base", 0.210);    // 210mm
        declare_parameter("ticks_per_rev", 330); // encoder

        wheel_radius_ = get_parameter("wheel_radius").as_double();
        wheel_base_   = get_parameter("wheel_base").as_double();
        ticks_per_rev_= get_parameter("ticks_per_rev").as_int();

        RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_base: %.3f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "ticks_per_rev: %d", ticks_per_rev_);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Init FastDDS subscriber
        encSub_.init();
        encSub_.set_callback([this](const WheelEncoderMsg& msg) {
            broadcast_odometry(msg.encLeft(), msg.encRight());
        });

        RCLCPP_INFO(this->get_logger(), "EncoderOdomNode initialized.");
        // encSub_.run();

    }

private:
    void broadcast_odometry(int32_t left_tick, int32_t right_tick)
    {
        static int32_t prev_left_tick = 0;
        static int32_t prev_right_tick = 0;
        static bool first_read_ = true;

        if (first_read_) {
            prev_left_tick = left_tick;
            prev_right_tick = right_tick;
            first_read_ = false;
            last_time_ = now();
            return;
        }

        // Δtick
        int64_t d_left_tick  = (int64_t)left_tick  - (int64_t)prev_left_tick;
        int64_t d_right_tick = (int64_t)right_tick - (int64_t)prev_right_tick;

        // Handle overflow
        if (llabs(d_left_tick) > (1LL << 30)) {
            d_left_tick = 0;
        }

        if (llabs(d_right_tick) > (1LL << 30)) {
            d_right_tick = 0;
        }

        // Update previous ticks
        prev_left_tick = left_tick;
        prev_right_tick = right_tick;

        // tick -> distance
        double dist_per_tick = 2.0 * M_PI * wheel_radius_ / ticks_per_rev_;
        double d_left = d_left_tick * dist_per_tick;
        double d_right = d_right_tick * dist_per_tick;
        double d_center = (d_left + d_right) / 2.0;
        double d_theta = (d_right - d_left) / wheel_base_;

        // dt
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // update pose
        theta_ += d_theta;
        x_ += d_center * cos(theta_);
        y_ += d_center * sin(theta_);

        // velocity
        double v = d_center / dt;
        double w = d_theta / dt;

        // publish odom
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;

        // ================== Set covariance ==================
        // pose.covariance: x, y, z, roll, pitch, yaw
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 1e-4;    // x
        odom_msg.pose.covariance[7] = 1.0;     // y (Can't measure, set high value)
        odom_msg.pose.covariance[14] = 99999;  // z (very high value for filter to ignore)
        odom_msg.pose.covariance[21] = 99999;  // roll
        odom_msg.pose.covariance[28] = 99999;  // pitch
        odom_msg.pose.covariance[35] = 2.5e-3; // yaw

        // twist.covariance: vx, vy, vz, vroll, vpitch, vyaw
        for (int i = 0; i < 36; i++) {
            odom_msg.twist.covariance[i] = 0.0;
        }
        odom_msg.twist.covariance[0] = 4e-4;    // vx
        odom_msg.twist.covariance[7] = 1.0;     // vy
        odom_msg.twist.covariance[14] = 99999;  // vz
        odom_msg.twist.covariance[21] = 99999;  // vroll
        odom_msg.twist.covariance[28] = 99999;  // vpitch
        odom_msg.twist.covariance[35] = 2.5e-3; // vyaw
        // ======================================================

        odom_pub_->publish(odom_msg);

        // broadcast tf
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    WheelEncoderMsgSubscriber encSub_;

    double wheel_radius_, wheel_base_;
    int ticks_per_rev_;

    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderOdomNode>());
    rclcpp::shutdown();
    return 0;
}
