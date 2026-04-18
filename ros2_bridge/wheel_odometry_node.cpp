#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>

class WheelOdometryNode : public rclcpp::Node {
public:
    WheelOdometryNode()
        : Node("wheel_odometry")
        , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
    {
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0335);
        wheel_separation_ = declare_parameter<double>("wheel_separation", 0.252);
        ticks_per_revolution_ = declare_parameter<double>("ticks_per_revolution", 1320.0);
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        publish_tf_ = declare_parameter<bool>("publish_tf", true);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        tick_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "wheel_encoder_ticks", 50,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) { onTicks(msg); });

        meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_revolution_;

        RCLCPP_INFO(
            get_logger(),
            "wheel_odometry started: r=%.5f m, L=%.5f m, ticks/rev=%.1f",
            wheel_radius_, wheel_separation_, ticks_per_revolution_);
    }

private:
    static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);
        return q;
    }

    void onTicks(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    double wheel_radius_;
    double wheel_separation_;
    double ticks_per_revolution_;
    double meters_per_tick_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    bool initialized_ = false;
    int32_t prev_left_ticks_ = 0;
    int32_t prev_right_ticks_ = 0;
    rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tick_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

void WheelOdometryNode::onTicks(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "wheel_encoder_ticks expects 2 values");
        return;
    }

    const int32_t left_ticks = msg->data[0];
    const int32_t right_ticks = msg->data[1];
    const rclcpp::Time now = this->now();

    if (!initialized_) {
        prev_left_ticks_ = left_ticks;
        prev_right_ticks_ = right_ticks;
        prev_stamp_ = now;
        initialized_ = true;
        return;
    }

    const int32_t d_left_ticks = left_ticks - prev_left_ticks_;
    const int32_t d_right_ticks = right_ticks - prev_right_ticks_;

    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;

    const double dt = (now - prev_stamp_).seconds();
    prev_stamp_ = now;
    if (dt <= 1e-6) {
        return;
    }

    // Convert ticks to distance
    const double d_left = static_cast<double>(d_left_ticks) * meters_per_tick_;
    const double d_right = static_cast<double>(d_right_ticks) * meters_per_tick_;
    const double d_center = 0.5 * (d_left + d_right);
    const double d_theta = (d_right - d_left) / wheel_separation_;

    // Update pose using a simple bicycle model
    const double theta_mid = theta_ + 0.5 * d_theta;
    x_ += d_center * std::cos(theta_mid);
    y_ += d_center * std::sin(theta_mid);
    theta_ += d_theta;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); // Normalize theta to [-pi, pi]

    // Compute velocities
    const double v = d_center / dt;
    const double w = d_theta / dt;

    // Publish odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = yawToQuaternion(theta_);
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = w;
    odom_pub_->publish(odom_msg);

    if (!publish_tf_) {
        return;
    }

    // Publish TF transform
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = yawToQuaternion(theta_);
    tf_broadcaster_->sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
