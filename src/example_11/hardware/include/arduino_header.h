#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <libserial/SerialPort.h>
#include <cstring>

class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  { 
    setup(serial_device, baud_rate, timeout_ms);
  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.IsOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  LibSerial::SerialPort serial_conn_;  ///< Underlying serial connection 
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H