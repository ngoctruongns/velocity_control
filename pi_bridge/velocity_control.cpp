#include "raw_packet_fastdds.hpp"
#include "serial_linux.hpp"
#include "process_data_packet.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr const char *kUplinkTopic = "robot.uart.uplink";
constexpr const char *kDownlinkTopic = "robot.uart.downlink";
constexpr size_t kMaxFrameBytes = 512;

class UartFrameCollector {
public:
    bool push(uint8_t byte, std::vector<uint8_t> &frame)
    {
        if (!in_frame_) {
            if (byte == STX) {
                in_frame_ = true;
                buffer_.clear();
                buffer_.push_back(byte);
            }
            return false;
        }

        if (byte == STX) {
            // Restart frame if previous frame was broken.
            buffer_.clear();
            buffer_.push_back(byte);
            return false;
        }

        buffer_.push_back(byte);
        if (buffer_.size() > kMaxFrameBytes) {
            in_frame_ = false;
            buffer_.clear();
            return false;
        }

        if (byte == ETX) {
            frame = buffer_;
            in_frame_ = false;
            buffer_.clear();
            return true;
        }

        return false;
    }

private:
    bool in_frame_ = false;
    std::vector<uint8_t> buffer_;
};
} // namespace

int main(int argc, char **argv)
{
    std::string uart_port = "/dev/mcu_uart";
    int uart_baudrate = 115200;

    if (argc >= 2) {
        uart_port = argv[1];
    }
    if (argc >= 3) {
        uart_baudrate = std::stoi(argv[2]);
    }

    SerialLinux serial(uart_port, uart_baudrate);
    if (!serial.isOpen()) {
        std::cerr << "Failed to open serial port: " << uart_port << "\n";
        return -1;
    }

    std::mutex serial_write_mtx;
    RawPacketPublisher uplink_pub;
    if (!uplink_pub.init(kUplinkTopic)) {
        std::cerr << "Failed to init FastDDS uplink publisher\n";
        return -2;
    }

    RawPacketSubscriber downlink_sub;
    if (!downlink_sub.init(kDownlinkTopic, [&](const std::vector<uint8_t> &bytes) {
            if (bytes.empty()) {
                return;
            }
            std::lock_guard<std::mutex> lock(serial_write_mtx);
            serial.writeData(bytes.data(), bytes.size());
        })) {
        std::cerr << "Failed to init FastDDS downlink subscriber\n";
        return -3;
    }

    std::cout << "pi_bridge started on " << uart_port << " @ " << uart_baudrate << "\n";

    UartFrameCollector collector;
    std::vector<uint8_t> frame;
    while (serial.isOpen()) {
        uint8_t byte = 0;
        int ret = serial.readData(&byte, 1);
        if (ret == 1 && collector.push(byte, frame)) {
            uplink_pub.publish(frame);
        } else if (ret <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    return 0;
}
