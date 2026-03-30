#include "process_data_packet.h"
#include "raw_packet_fastdds.hpp"
#include "velocity_control.h"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {
constexpr const char *kUplinkTopic = "robot.uart.uplink";
constexpr const char *kDownlinkTopic = "robot.uart.downlink";

struct MotorRpmType {
    uint8_t type;
    int16_t left_rpm;
    int16_t right_rpm;
} __attribute__((packed));

int16_t clampInt16(double value)
{
    if (value > static_cast<double>(std::numeric_limits<int16_t>::max())) {
        return std::numeric_limits<int16_t>::max();
    }
    if (value < static_cast<double>(std::numeric_limits<int16_t>::min())) {
        return std::numeric_limits<int16_t>::min();
    }
    return static_cast<int16_t>(std::lround(value));
}
} // namespace

class Ros2BridgeNode : public rclcpp::Node {
public:
    Ros2BridgeNode()
        : Node("ros2_bridge_node")
    {
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);
        wheel_separation_ = declare_parameter<double>("wheel_separation", 0.25);

        wheel_enc_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("wheel_encoder_ticks", 10);
        motor_rpm_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("motor_rpm_feedback", 10);
        debug_pub_ = create_publisher<std_msgs::msg::String>("mcu_debug", 10);

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&Ros2BridgeNode::onCmdVel, this, std::placeholders::_1));

        pid_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "pid_config", 10,
            std::bind(&Ros2BridgeNode::onPidConfig, this, std::placeholders::_1));

        comm_ctrl_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "comm_ctrl", 10,
            std::bind(&Ros2BridgeNode::onCommCtrl, this, std::placeholders::_1));

        led_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "led_control", 10,
            std::bind(&Ros2BridgeNode::onLedControl, this, std::placeholders::_1));

        buzzer_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "buzzer_control", 10,
            std::bind(&Ros2BridgeNode::onBuzzerControl, this, std::placeholders::_1));

        if (!downlink_pub_.init(kDownlinkTopic)) {
            throw std::runtime_error("Failed to initialize FastDDS downlink publisher");
        }

        if (!uplink_sub_.init(
                kUplinkTopic,
                std::bind(&Ros2BridgeNode::onUplinkFrame, this, std::placeholders::_1))) {
            throw std::runtime_error("Failed to initialize FastDDS uplink subscriber");
        }

        RCLCPP_INFO(get_logger(), "ros2_bridge is running");
    }

private:
    template <typename T>
    void sendPacket(const T &packet)
    {
        std::array<uint8_t, BUFFER_SIZE> tx_buffer{};
        const auto *raw = reinterpret_cast<const uint8_t *>(&packet);
        uint8_t tx_len = encoderAllPackage(raw, static_cast<uint8_t>(sizeof(T)), tx_buffer.data());
        std::vector<uint8_t> frame(tx_buffer.begin(), tx_buffer.begin() + tx_len);
        downlink_pub_.publish(frame);
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double v = msg->linear.x;
        const double w = msg->angular.z;

        const double left_vel = v - (w * wheel_separation_ * 0.5);
        const double right_vel = v + (w * wheel_separation_ * 0.5);

        const double left_rpm = (left_vel / (2.0 * M_PI * wheel_radius_)) * 60.0;
        const double right_rpm = (right_vel / (2.0 * M_PI * wheel_radius_)) * 60.0;

        CmdVelType cmd{};
        cmd.type = CMD_VEL_COMMAND;
        cmd.left_rpm = clampInt16(left_rpm);
        cmd.right_rpm = clampInt16(right_rpm);
        sendPacket(cmd);
    }

    void onPidConfig(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        PIDConfigType cmd{};
        cmd.type = PID_CONFIG_COMMAND;
        cmd.Kp = static_cast<float>(msg->x);
        cmd.Ki = static_cast<float>(msg->y);
        cmd.Kd = static_cast<float>(msg->z);
        sendPacket(cmd);
    }

    void onCommCtrl(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        // CommCtrlType in velocity_control.h has only one field, so we send [type, feedback].
        std::array<uint8_t, 2> payload{};
        payload[0] = COMM_CTRL_COMMAND;
        payload[1] = msg->data;

        std::array<uint8_t, BUFFER_SIZE> tx_buffer{};
        uint8_t tx_len = encoderAllPackage(payload.data(), static_cast<uint8_t>(payload.size()), tx_buffer.data());
        std::vector<uint8_t> frame(tx_buffer.begin(), tx_buffer.begin() + tx_len);
        downlink_pub_.publish(frame);
    }

    void onLedControl(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 5) {
            RCLCPP_WARN(get_logger(), "led_control expects [r, g, b, param1, param2]");
            return;
        }

        LEDControlType cmd{};
        cmd.type = LED_CONTROL_COMMAND;
        cmd.r = static_cast<uint8_t>(std::min<uint16_t>(msg->data[0], 255));
        cmd.g = static_cast<uint8_t>(std::min<uint16_t>(msg->data[1], 255));
        cmd.b = static_cast<uint8_t>(std::min<uint16_t>(msg->data[2], 255));
        cmd.param1 = msg->data[3];
        cmd.param2 = msg->data[4];
        sendPacket(cmd);
    }

    void onBuzzerControl(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(get_logger(), "buzzer_control expects [param1, param2]");
            return;
        }

        BuzzerControlType cmd{};
        cmd.type = BUZZER_CONTROL_COMMAND;
        cmd.param1 = msg->data[0];
        cmd.param2 = msg->data[1];
        sendPacket(cmd);
    }

    void onUplinkFrame(const std::vector<uint8_t> &frame)
    {
        if (frame.empty()) {
            return;
        }

        static std::array<uint8_t, BUFFER_SIZE> decoded{};
        for (uint8_t byte : frame) {
            uint8_t len = handleRxByteConcurrent(byte, decoded.data());
            if (len > 0) {
                decodePacket(decoded.data(), len);
            }
        }
    }

    void decodePacket(const uint8_t *buf, uint8_t len)
    {
        const uint8_t type = buf[0];
        switch (type) {
            case DEBUG_STRING: {
                std::string text;
                if (len > 1) {
                    text.assign(reinterpret_cast<const char *>(&buf[1]), reinterpret_cast<const char *>(&buf[len]));
                }
                std_msgs::msg::String msg;
                msg.data = text;
                debug_pub_->publish(msg);
                break;
            }
            case WHEEL_ENC_COMMAND: {
                if (len != sizeof(WheelEncType)) {
                    RCLCPP_WARN(get_logger(), "Invalid WHEEL_ENC_COMMAND length: %u", len);
                    break;
                }
                WheelEncType data{};
                std::memcpy(&data, buf, sizeof(WheelEncType));

                std_msgs::msg::Int32MultiArray msg;
                msg.data = {data.left_enc, data.right_enc};
                wheel_enc_pub_->publish(msg);
                break;
            }
            case MOTOR_RPM_COMMAND: {
                if (len != sizeof(MotorRpmType)) {
                    RCLCPP_WARN(get_logger(), "Invalid MOTOR_RPM_COMMAND length: %u", len);
                    break;
                }
                MotorRpmType data{};
                std::memcpy(&data, buf, sizeof(MotorRpmType));

                std_msgs::msg::Int16MultiArray msg;
                msg.data = {data.left_rpm, data.right_rpm};
                motor_rpm_pub_->publish(msg);
                break;
            }
            default:
                RCLCPP_DEBUG(get_logger(), "Unhandled packet type: %u", type);
                break;
        }
    }

    double wheel_radius_;
    double wheel_separation_;

    RawPacketPublisher downlink_pub_;
    RawPacketSubscriber uplink_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_enc_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr motor_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr comm_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr led_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr buzzer_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<Ros2BridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception &ex) {
        std::cerr << "Failed to start ros2_bridge: " << ex.what() << '\n';
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
