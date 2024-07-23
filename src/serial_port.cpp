#include <async_serial/serial_port.hpp>

#include <rclcpp/logging.hpp>

namespace async_serial {

// SerialPort::SerialPort()
// {
//   SerialPort("/dev/ttyUSB0", 115200);
// }

SerialPort::SerialPort(
    const std::string& port,
    uint32_t baud_rate
)
: io_service_(new boost::asio::io_service()), serial_(*io_service_), port_(port), baud_rate_(baud_rate) {
  rx_buf_.resize(BUF_SIZE);
}

SerialPort::~SerialPort() {
  if (is_open()) {
    close();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool SerialPort::is_open() const {
  return serial_.is_open();
}

bool SerialPort::open() {
  try {
    serial_.open(port_);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  }
  catch (const boost::system::system_error& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::open"), "cannot open port " << port_ << ": " << e.what());
    return false;
  }
  return is_open();
}

void SerialPort::close() {
  try {
    serial_.close();
  }
  catch (const boost::system::system_error& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::close"), "cannot close port" << port_ << ": " << e.what());
  }
}

size_t SerialPort::send(const std::vector<uint8_t>& buf) {
  return serial_.write_some(boost::asio::buffer(buf.data(), buf.size()));
}

void SerialPort::async_send(const std::vector<uint8_t>& buf) {
  serial_.async_write_some(
    boost::asio::buffer(buf),
    [this](boost::system::error_code error, size_t bytes_transferred) {
      async_send_handler(error, bytes_transferred);
    }
  );
}

void SerialPort::add_receive_callback(SerialCallback fn) {
  func_ = fn;
  serial_.async_read_some(
    boost::asio::buffer(rx_buf_),
    [this](boost::system::error_code error, size_t bytes_transferred) {
      async_receive_handler(error, bytes_transferred);
    }
  );
  std::thread t {
    [this]() {
      io_service_->run();
    }
  };
  thread_.swap(t);
}

void SerialPort::async_send_handler(const boost::system::error_code& err, size_t bytes_transferred) {
  (void)bytes_transferred;
  if (err) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::async_send_handler"), err.message());
    (void)bytes_transferred;
  }
}

void SerialPort::async_receive_handler(const boost::system::error_code& err, size_t bytes_received) {
  if (err) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::async_receive_handler"), err.message());
    close();
    return;
  }

  if (bytes_received > 0 && func_) {
    func_(rx_buf_, bytes_received);
    serial_.async_read_some(
      boost::asio::buffer(rx_buf_),
      [this](boost::system::error_code error, size_t bytes_transferred) {
        async_receive_handler(error, bytes_transferred);
      }
    );
  }
}

}