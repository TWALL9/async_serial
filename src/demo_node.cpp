#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "async_serial/serial_port.hpp"
using namespace std::chrono_literals;

class DemoNode : public rclcpp::Node {
public:
  DemoNode() : Node("DemoNode"), port_("/dev/ttyACM1", 115200) {
    pub_ = this->create_publisher<std_msgs::msg::String>("serial_received", 10);
    port_.open();

    port_.add_receive_callback(std::bind(
      &DemoNode::serial_callback, this, std::placeholders::_1, std::placeholders::_2
    ));

    timer_ = this->create_wall_timer(
      500ms, std::bind(&DemoNode::timer_callback, this)
    );
  }

private:
  void serial_callback(const std::vector<uint8_t>& buf, size_t bytes_received) {
    (void)bytes_received;
    std::string received(buf.begin(), buf.end());
    auto msg = std_msgs::msg::String();
    msg.data = received;
    pub_->publish(msg);
  }

  void timer_callback() {
    std::vector<uint8_t> send_buf {'j', 'k', 'l'};
    port_.send(send_buf);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  async_serial::SerialPort port_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoNode>());
  rclcpp::shutdown();

  return 0;
}