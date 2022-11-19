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
#include "week10_hw/msg/details.hpp"

using std::placeholders::_1;
/**
 * @brief This is custom subscriber class
 *
 */
class MySubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new My Subscriber object
   *
   */
  MySubscriber() : Node("MySubscriber") {
    subscription_ = this->create_subscription<week10_hw::msg::Details>(
        "publish_details", 10,
        std::bind(&MySubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Subscriber call back function
   *
   * @param msg
   */
  void topic_callback(const week10_hw::msg::Details& msg) const {
    RCLCPP_INFO(this->get_logger(),
                "Received Details \n First Name: %s "
                "\nLast Name: %s "
                "\nAge: %d ",
                msg.first_name.c_str(), msg.last_name.c_str(), msg.age);
  }
  // subscriber pointer
  rclcpp::Subscription<week10_hw::msg::Details>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
