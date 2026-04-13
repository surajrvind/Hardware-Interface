#include "ros2_control_demo_example_11/arduino_comms.hpp"
#include <sstream>

void ArduinoComms::connect(const std::string &device, int32_t baud_rate, int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;

    serial_conn_.Open(device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void ArduinoComms::disconnect()
{
    if (serial_conn_.IsOpen()){
        serial_conn_.Close();
    };
}

bool ArduinoComms::connected() const
{
    return serial_conn_.IsOpen();
}

std::string ArduinoComms::send_msg(const std::string &msg_to_send, bool)
{
    serial_conn_.Write(msg_to_send);
    //std::cout << "TX: " << msg_to_send << std::endl;
    return "";
}

std::string ArduinoComms::read_line()
{
    std::string result;

    try
    {
        if(serial_conn_.IsDataAvailable()){
            serial_conn_.ReadLine(result, '\n', timeout_ms_);
        }
    }
    catch (...)
    {
        // ignore read timeout or serial issues
    }

    return result;
}

uint8_t compute_checksum(const std::string &data){
    uint8_t sum = 0;
    for (char c : data){
        sum += static_cast<uint8_t>(c);
    }
    return sum;
}

void ArduinoComms::set_motor_values(int throttle, float steering)
{
    // std::stringstream payload;
    // payload << "C," << throttle << "," << steering;

    // std::string payload_str = payload.str();
    // uint8_t checksum = compute_checksum(payload_str);

    // std::stringstream packet;
    // packet << "@" << payload_str << "," << static_cast<int>(checksum) << "\n";
    // send_msg(packet.str());
    // std::cout << "TX PACKET: " << packet.str() << std::endl;

    std::stringstream ss;
    ss << throttle << "," << steering << "\n";
    send_msg(ss.str());
}

void ArduinoComms::send_empty_msg(const std::string &, bool)
{
    serial_conn_.Write("\n");
}

LibSerial::BaudRate ArduinoComms::convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        default: throw std::runtime_error("Unsupported baud rate");
    }
}