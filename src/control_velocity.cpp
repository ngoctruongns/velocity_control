#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"  // publish [left_rpm, right_rpm]

#include "VelocityMsgPublisher.h"  // FastDDS publisher

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController() : Node("diff_drive_controller")
    {
        this->declare_parameter("wheel_radius", 0.05); // 5cm
        this->declare_parameter("wheel_base", 0.30);   // 30cm
        this->declare_parameter("cmd_topic", "cmd_vel");

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_   = this->get_parameter("wheel_base").as_double();
        std::string cmd_topic = this->get_parameter("cmd_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_base: %.3f m", wheel_base_);

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic, 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->cmdVelCallback(msg);
            });

        pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("wheel_rpm", 10);

        // Init fastDDS publisher
        velPub_.init();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (velPub_.getMatched() == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "No subscribers to MotorControlMsg. Please check connection.");
            return;
        }
        double v = msg->linear.x;   // m/s
        double w = msg->angular.z;  // rad/s

        // Calculate for wheel speed (m/s)
        double v_left  = v - (wheel_base_/2.0) * w;
        double v_right = v + (wheel_base_/2.0) * w;

        // Convert to rad/s
        double w_left  = v_left / wheel_radius_;
        double w_right = v_right / wheel_radius_;

        // Convert to RPM
        int rpm_left  = static_cast<int>(w_left  * 60.0 / (2.0 * M_PI));
        int rpm_right = static_cast<int>(w_right * 60.0 / (2.0 * M_PI));

        std_msgs::msg::Int32MultiArray rpm_msg;
        rpm_msg.data = {rpm_left, rpm_right};
        pub_->publish(rpm_msg);

        RCLCPP_INFO(this->get_logger(), "cmd_vel(v=%.2f, w=%.2f) -> RPM[L=%d, R=%d]",
                    v, w, rpm_left, rpm_right);

        // Publish via FastDDS
        MotorControlMsg motor_msg;
        motor_msg.leftRpm(rpm_left);
        motor_msg.rightRpm(rpm_right);
        velPub_.pushlishMessageData(motor_msg);
    }

    double wheel_radius_;
    double wheel_base_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    VelocityMsgPublisher velPub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffDriveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
