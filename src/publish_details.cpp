/**
 * @file publish_details.cpp
 * @author Koundinya Vinnakota
 * @brief This the publisher and servie provider
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "week10_hw/msg/details.hpp"
#include "week10_hw/srv/details.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"



using namespace std::chrono_literals;
/**
 * @brief This is the publisher class
 *
 */
class DetailsPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Details Publisher object
   *
   */
  DetailsPublisher() : Node("details_publisher") {

    // Declaring and acquiring topic names and publishing speed
    this->declare_parameter("Name_of_topic", "publish_details");
    this->declare_parameter("publishing_speed", 500);
    //Intialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // Initial message variables declaration
    message_.first_name = "Koundinya";
    message_.last_name = "Vinnakota";
    message_.age = 22;
    // Creation of publisher
    details_publisher_ = this->create_publisher<week10_hw::msg::Details>(
        this->get_parameter("Name_of_topic").as_string(), 10);
    // // Creation of service
    // service_ = this->create_service<week10_hw::srv::Details>(
    //     "change_details",
    //     std::bind(&DetailsPublisher::change_details, this,
    //               std::placeholders::_1, std::placeholders::_2));
    // Creation of timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publishing_speed").as_int()),
        std::bind(&DetailsPublisher::timer_callback, this));
    // Setting logger level to Debug
    if (rcutils_logging_set_logger_level(
            this->get_name(),
            RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK)
      RCLCPP_DEBUG(this->get_logger(), "Started with DEBUG");
    else
      RCLCPP_INFO(this->get_logger(), "Started without DEBUG");
    // Logging statement for debug
    RCLCPP_DEBUG(this->get_logger(), " Publisher node started ");
    // Logging statement for error level
    if (this->get_parameter("publishing_speed").as_int() < 1000)
      RCLCPP_ERROR(this->get_logger(), "The speed of publish is too high!");
    // logging statement for fatal level
    if (this->get_parameter("publishing_speed").as_int() < 500) {
      RCLCPP_FATAL(this->get_logger(),
                   "The speed of publish is extremely high! Quiting.");
    }
  }

 private:
  /**
   * @brief timer_callback is a callback function for the publisher
   *
   */
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(),
                " Details \n First Name: %s "
                "\nLast Name: %s "
                "\nAge: %d ",
                message_.first_name.c_str(), message_.last_name.c_str(),
                message_.age);

    // Setting the messages for transformation
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "talk";
    t.child_frame_id = "world";
    t.transform.translation.x = 2.5;
    t.transform.translation.y = 3.5;
    t.transform.translation.z = 0.5;

    tf2::Quaternion q;
    q.setRPY(1.57, 1.57,1.57);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    
    details_publisher_->publish(message_);
  }
  /**
   * @brief Server call back function
   *
   * @param request Request to service
   * @param response Response from service
   */
  void change_details(
      const std::shared_ptr<week10_hw::srv::Details::Request> request,
      std::shared_ptr<week10_hw::srv::Details::Response> response) {
    // Response variables getting assigned
    response->changed_first_name = request->first_name;
    response->changed_last_name = request->last_name;
    response->changed_age = request->age;
    // logging statement for warning
    RCLCPP_WARN(this->get_logger(), "Details Changed ");
    message_.first_name = request->first_name;
    message_.last_name = request->last_name;
    message_.age = request->age;
    // logging statements
    RCLCPP_INFO(this->get_logger(),
                " Change Details Request \n First Name: %s "
                "\nLast Name: %s "
                "\nAge: %d ",
                request->first_name.c_str(), request->last_name.c_str(),
                request->age);
    RCLCPP_INFO(this->get_logger(),
                " Changed Details Request \n First Name: %s "
                "\nLast Name: %s "
                "\nAge: %d ",
                response->changed_first_name.c_str(),
                response->changed_last_name.c_str(), response->changed_age);
  }
  // decalration of private variables
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<week10_hw::msg::Details>::SharedPtr details_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  week10_hw::msg::Details message_;  // Custom message to publish
  rclcpp::Service<week10_hw::srv::Details>::SharedPtr
      service_;  // service pointer
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetailsPublisher>());
  rclcpp::shutdown();
  return 0;
}
