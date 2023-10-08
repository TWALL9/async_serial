#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <vector>

namespace async_serial
{

using SerialCallback = std::function<void (std::vector<uint8_t> &, const size_t &)>;
using boost_serial = boost::asio::serial_port_base;

class SerialPort
{

public:
  // SerialPort();

  SerialPort(
    const std::string& port,
    uint32_t baud_rate
  );

  ~SerialPort();

  bool open();
  void close();

  bool isOpen() const;

  size_t send(const std::vector<uint8_t>& buf);
  void asyncSend(const std::vector<uint8_t>& buf);

  void addReceiveCallback(SerialCallback fn);

private:
  void asyncSendHandler(const boost::system::error_code& err, size_t bytes_transferred);
  void asyncReceiveHandler(const boost::system::error_code& err, size_t bytes_received);

  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_;
  std::string port_;
  uint32_t baud_rate_;
  std::vector<uint8_t> rx_buf_;
  SerialCallback func_;
  static constexpr size_t BUF_SIZE {2048};
};

}