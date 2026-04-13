#pragma once

#include <sstream>
#include <iostream>
#include <libserial/SerialPort.h>

class ArduinoComms {
    public:
        ArduinoComms() = default;

        void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

        void disconnect();

        bool connected() const;

        std::string send_msg(const std::string &msg_to_send, bool print_output = false);

        std::string read_line();

        void set_motor_values(int throttle, float steering);

        void send_empty_msg(const std::string&, bool print_output = false);

    private:
        LibSerial::BaudRate convert_baud_rate(int baud_rate);
        LibSerial::SerialPort serial_conn_;
        int timeout_ms_;
};