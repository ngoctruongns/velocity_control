#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

class Ps3UtilsControlNode : public rclcpp::Node
{
public:
    Ps3UtilsControlNode() : Node("ps3_utils_control_node")
    {
        joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
        led_topic_ = declare_parameter<std::string>("led_topic", "led_control");
        buzzer_topic_ = declare_parameter<std::string>("buzzer_topic", "buzzer_control");

        led_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(led_topic_, 10);
        buzzer_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(buzzer_topic_, 10);

        loadBindingsFromParameters();

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, 20, std::bind(&Ps3UtilsControlNode::onJoy, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "ps3_utils_control is running: joy='%s', led='%s', buzzer='%s', bindings=%zu",
                    joy_topic_.c_str(), led_topic_.c_str(), buzzer_topic_.c_str(),
                    bindings_.size());
    }

private:
    enum class Target { kLed, kBuzzer };

    struct Binding {
        std::string name;
        int button_index{-1};
        bool trigger_on_press{true};
        Target target{Target::kLed};
        std::vector<uint16_t> command;
        bool last_pressed{false};
    };

    static std::vector<uint16_t> clampToUInt16(const std::vector<int64_t> &src)
    {
        std::vector<uint16_t> out;
        out.reserve(src.size());
        for (const int64_t v : src) {
            const auto clamped = std::clamp<int64_t>(v, 0, 65535);
            out.push_back(static_cast<uint16_t>(clamped));
        }
        return out;
    }

    void loadBindingsFromParameters()
    {
        const auto mapping_names =
            declare_parameter<std::vector<std::string>>("mappings", std::vector<std::string>{});

        for (const auto &name : mapping_names) {
            const std::string prefix = name + ".";
            const auto button = declare_parameter<int64_t>(prefix + "button", -1);
            const auto target_str = declare_parameter<std::string>(prefix + "target", "led");
            const auto trigger_on_press =
                declare_parameter<bool>(prefix + "trigger_on_press", true);
            const auto command_raw =
                declare_parameter<std::vector<int64_t>>(prefix + "command", std::vector<int64_t>{});

            Binding b;
            b.name = name;
            b.button_index = static_cast<int>(button);
            b.trigger_on_press = trigger_on_press;
            b.command = clampToUInt16(command_raw);

            if (target_str == "led") {
                b.target = Target::kLed;
                if (b.command.size() < 6) {
                    RCLCPP_WARN(get_logger(),
                                "binding '%s' has target=led but command size=%zu (<6), skipped",
                                b.name.c_str(), b.command.size());
                    continue;
                }
            } else if (target_str == "buzzer") {
                b.target = Target::kBuzzer;
                if (b.command.size() < 3) {
                    RCLCPP_WARN(get_logger(),
                                "binding '%s' has target=buzzer but command size=%zu (<3), skipped",
                                b.name.c_str(), b.command.size());
                    continue;
                }
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "binding '%s' has invalid target='%s', expected 'led' or 'buzzer', skipped",
                    b.name.c_str(), target_str.c_str());
                continue;
            }

            if (b.button_index < 0) {
                RCLCPP_WARN(get_logger(), "binding '%s' has invalid button index=%d, skipped",
                            b.name.c_str(), b.button_index);
                continue;
            }

            bindings_.push_back(std::move(b));
        }
    }

    void publishCommand(Target target, const std::vector<uint16_t> &command)
    {
        std_msgs::msg::UInt16MultiArray msg;
        msg.data = command;

        if (target == Target::kLed) {
            led_pub_->publish(msg);
        } else {
            buzzer_pub_->publish(msg);
        }
    }

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        for (auto &binding : bindings_) {
            const bool valid_index =
                binding.button_index >= 0 &&
                static_cast<size_t>(binding.button_index) < msg->buttons.size();
            if (!valid_index) {
                continue;
            }

            const bool pressed = msg->buttons[static_cast<size_t>(binding.button_index)] != 0;
            bool should_fire = false;

            if (binding.trigger_on_press) {
                should_fire = pressed && !binding.last_pressed;
            } else {
                should_fire = !pressed && binding.last_pressed;
            }

            if (should_fire) {
                publishCommand(binding.target, binding.command);
            }

            binding.last_pressed = pressed;
        }
    }

    std::string joy_topic_;
    std::string led_topic_;
    std::string buzzer_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr led_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr buzzer_pub_;

    std::vector<Binding> bindings_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ps3UtilsControlNode>());
    rclcpp::shutdown();
    return 0;
}
