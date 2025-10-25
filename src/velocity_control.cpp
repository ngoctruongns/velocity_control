#include "VelocityMsgPublisher.h"
#include "VelocityMsgSubscriber.h"
#include "WheelEncoderMsgPublisher.h"
#include "WheelEncoderMsgSubscriber.h"
#include "velocity_control.h"
#include "process_data_packet.h"
#include "serial_linux.hpp"

/*******          Global variable        ************/
uint8_t rx_buffer[BUFFER_SIZE] = {0};
uint8_t tx_buffer[BUFFER_SIZE] = {0};


// Handle wheel encoder package
void wheelEncoderHandler(const uint8_t *buf, uint8_t len, WheelEncoderMsgPublisher &mypub)
{
    WheelEncType wheel_enc;
    uint8_t pack_len = sizeof(WheelEncType);

    if (len != pack_len) {
        std::cerr << "Invalid WHEEL_ENC_COMMAND package size.\n";
        return;
    }

    // Copy data to struct
    memcpy(&wheel_enc, buf, pack_len);

    // Publish message
    WheelEncoderMsg msg;
    msg.encLeft(wheel_enc.left_enc);
    msg.encRight(wheel_enc.right_enc);
    mypub.pushlishMessageData(msg);
    std::cout << "Enc_Left: " << wheel_enc.left_enc << ", Enc_Right: " << wheel_enc.right_enc
              << std::endl;
}

// Handle debug string package
void debugStringHandler(const uint8_t *buf, uint8_t len)
{
    char str[BUFFER_SIZE];
    memcpy(str, buf, len); // Skip type byte
    str[len] = '\0';       // Null-terminate
    std::cout << "UART_MSG: " << str << std::endl;
}

int main(int argc, char **argv)
{
    // Get uart port and baudrate from arguments or config file
    std::string uart_port = "/dev/arduino_uno";
    int uart_baudrate = 19200;

    if (argc >= 2) {
        uart_port = argv[1];
    }
    if (argc >= 3) {
        uart_baudrate = std::stoi(argv[2]);
    }

    // Print to debug size of data types
    std::cout << "Size of CmdVelType: " << sizeof(CmdVelType) << " bytes\n";
    std::cout << "Size of WheelEncType: " << sizeof(WheelEncType) << " bytes\n";

    // Creat serial port object
    // Add rules to set arduino serial port name
    SerialLinux serial(uart_port, uart_baudrate);
    if (!serial.isOpen()) {
        std::cerr << "Failed to open serial port.\n";
        return -1;
    }
    // Create publisher
    VelocityMsgSubscriber mysub;
    if (mysub.init()) {
        mysub.set_callback([&serial](const MotorControlMsg &msg) {
            CmdVelType cmd_vel{}; // initialize all members to zero

            // Cast int32_t to int16_t to send only lower 16 bits
            cmd_vel.type = CMD_VEL_COMMAND;
            cmd_vel.left_rpm = static_cast<int16_t>(msg.leftRpm());
            cmd_vel.right_rpm = static_cast<int16_t>(msg.rightRpm());

            // // Print to debug
            // std::cout << "Sent CMD_VEL - Left RPM: " << cmd_vel.left_rpm
            //           << ", Right RPM: " << cmd_vel.right_rpm << std::endl;

            // Fill buffer data
            uint8_t tx_len = encoderAllPackage(reinterpret_cast<const uint8_t *>(&cmd_vel),
                                               sizeof(CmdVelType), tx_buffer);

            // Write serial data
            serial.writeData(tx_buffer, tx_len);
        });
    } else {
        std::cerr << "Failed to initialize subscriber.\n";
        return -1;
    }

    // While loop to check receive data and send wheel encoders
    // Publish encoder values
    WheelEncoderMsgPublisher mypub;
    if (!mypub.init()) {
        std::cerr << "Failed to initialize publisher.\n";
        return -2;
    }

    while (serial.isOpen()) {
        uint8_t byte;

        if (serial.readData(&byte, 1) == 1) {
            uint8_t rx_len = handleRxByteConcurrent(byte, rx_buffer);
            if (rx_len > 0) {
                // A complete package is received
                uint8_t package_type = rx_buffer[0]; // First byte is type
                switch (package_type) {
                    case DEBUG_STRING:
                        debugStringHandler(rx_buffer, rx_len);
                        break;
                    case CMD_VEL_COMMAND:
                        // Not expected to receive this type
                        std::cerr << "Received unexpected CMD_VEL_COMMAND package.\n";
                        break;
                    case WHEEL_ENC_COMMAND:
                        wheelEncoderHandler(rx_buffer, rx_len, mypub);
                        break;
                    default:
                        std::cerr << "Unknown package type: " << static_cast<int>(package_type)
                                  << std::endl;
                        break;
                }
            }
        }
    }

    return 0;
}
