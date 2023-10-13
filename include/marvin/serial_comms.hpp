#pragma once
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

std::vector<std::string> split(const std::string& input, char delimiter) {
    std::vector<std::string> answer;
    std::stringstream ss(input);
    std::string temp; 
    while (std::getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }
    return answer;
}

class SerialPort
{
  LibSerial::SerialPort serial;
  int32_t timeout_ms_;
  std::tuple<int, int, int, int> init_encoder = {0, 0, 0, 0};
public:
  void connect(const std::string& port, int baud_rate, int timeout_ms) {
    RCLCPP_INFO(
      rclcpp::get_logger("SerialPort"),
      "Port: %s, Baudrate: %d", 
      port.c_str(), baud_rate);
    timeout_ms_ = timeout_ms;
    serial.Open(port.c_str());
    serial.SetBaudRate(convert_baud_rate(baud_rate));
    init_encoder = get_encoder_value();
  }

  void disconnect() {
    serial.Close();
  }

  bool connected() {
    return serial.IsOpen();
  }

  void send_msg(const std::string &msg_to_send)
  {
    serial.FlushIOBuffers(); // Just in case
    serial.Write(msg_to_send);
  }

  std::string read_msg(char line_terminator = '\r')
  {
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial.ReadLine(response, line_terminator, timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }
    return response;
  }

  std::tuple<int, int, int, int> get_encoder_value() {
    std::stringstream ss;
    ss << "?C 1_?C 2_?S 1_?S 2" << char(0x0d);
    send_msg(ss.str());

    const auto q1 = read_msg();
    const auto r1 = read_msg();
    const auto q2 = read_msg();
    const auto r2 = read_msg();
    const auto q3 = read_msg();
    const auto r3 = read_msg();
    const auto q4 = read_msg();
    const auto r4 = read_msg();

    try
    {  
      const auto [e1, e2, s1, s2] = init_encoder;
      const int encoder_1 = std::stoi(split(r1, '=').back()) - e1;
      const int encoder_2 = std::stoi(split(r2, '=').back()) - e2;
      const int speed_1 = std::stoi(split(r3, '=').back()) - s1;
      const int speed_2 = std::stoi(split(r4, '=').back()) - s2;
      return {encoder_1, encoder_2, speed_1, speed_2};
    }
    catch(const std::exception& e)
    {
      return {-1, -1, -1, -1}; // todo 
    }
  }

  // -1000 ~ 1000 %
  bool set_motor_value(int v1, int v2) {
    std::stringstream ss;
    ss << "!G 1 "<< v1 << "_!G 2 " << v2 << char(0x0d);
    send_msg(ss.str());
    auto response = read_msg();
    return true;
  }

  void set_max_motor_rpm(int v1, int v2) {
    std::stringstream ss;
    ss << "^MMOD 1 1_^MMOD 2 1_^MXRPM 1 " << v1 << "_^MXRPM 2 " << v2 << char(0x0d);
    send_msg(ss.str());
    auto response = read_msg();
  }
};
