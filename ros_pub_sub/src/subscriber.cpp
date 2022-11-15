/**
 * @file subscriber.cpp
 * @author Koundinya Vinnakota
 * @brief This is a sample subscriber
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
/**
 * @brief This is custom subscriber class
 *
 */
class MySubscriber : public rclcpp::Node {
 public:
  MySubscriber() : Node("MySubscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MySubscriber::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard the message Mr. K: '%s'",
    msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
