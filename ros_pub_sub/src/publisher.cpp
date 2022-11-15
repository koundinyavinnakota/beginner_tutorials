/**
 * @file Publisher.cpp
 * @author Koundinya Vinnakota
 * @brief This is a publisher example
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief This is the sample publisher class to publish a message
 *
 */
class MyPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor of the MyPublisher class, initializing a node and
   * counter to 0
   *
   */
  MyPublisher() : Node("MyPublisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms,
     std::bind(&MyPublisher::timer_callback, this));
  }

 private:
  /**
   * @brief timer_callback is a callback function for the publisher
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, Koundinya ! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}
