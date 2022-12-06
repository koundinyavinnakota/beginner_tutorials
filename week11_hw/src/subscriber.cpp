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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "week11_hw/msg/details.hpp"

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
    subscription_ = this->create_subscription<week11_hw::msg::Details>(
        "publish_details", 10,
        std::bind(&MySubscriber::topic_callback, this, _1));

    // target_frame_ =
    // this->declare_parameter<std::string>("target_frame","talk"); tf_buffer_ =
    // std::make_unique<tf2_ros::Buffer>(this->get_clock()); tf_listener_ =
    // std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

 private:
  /**
   * @brief Subscriber call back function
   *
   * @param msg
   */
  void topic_callback(const week11_hw::msg::Details& msg) const {
    geometry_msgs::msg::TransformStamped t;
    // std::string fromFrameRel = target_frame_.c_str();
    // std::string toFrameRel = "listener";
    RCLCPP_INFO(this->get_logger(),
                "Received Details \n First Name: %s "
                "\nLast Name: %s "
                "\nAge: %d ",
                msg.first_name.c_str(), msg.last_name.c_str(), msg.age);
    // try{
    //   t =
    //   tf_buffer_->lookupTransform(toFrameRel,fromFrameRel,tf2::TimePointZero);

    // }catch(const tf2::TransformException &ex){
    //   RCLCPP_INFO(this->get_logger(),"Could not transform %s to %s:
    //   %s",toFrameRel.c_str(),fromFrameRel.c_str(),ex.what()); return;
    // }
  }
  // subscriber pointer
  rclcpp::Subscription<week11_hw::msg::Details>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
