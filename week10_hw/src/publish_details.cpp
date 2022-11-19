#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "week10_hw/msg/details.hpp"
#include "week10_hw/srv/details.hpp"
// #include "week10_hw"

using namespace std::chrono_literals;

class DetailsPublisher : public rclcpp::Node {
public:
  DetailsPublisher() : Node("details_publisher") {
    week10_hw::msg::Details temp_;
    this->declare_parameter("Name_of_topic", "publish_details");
    this->declare_parameter("publishing_speed", 500);

    message_.first_name = "Koundinya";
    message_.last_name = "Vinnakota";
    message_.age = 22;

    details_publisher_ =
        this->create_publisher<week10_hw::msg::Details>(this->get_parameter("Name_of_topic").as_string(), 10);

    service_ = this->create_service<week10_hw::srv::Details>(
        "change_details",
        std::bind(&DetailsPublisher::change_details, this,
                  std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("publishing_speed").as_int()), std::bind(&DetailsPublisher::timer_callback, this));

    if (rcutils_logging_set_logger_level(
            this->get_name(),
            RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK)
      RCLCPP_DEBUG(this->get_logger(), "Started with DEBUG");
    else
      RCLCPP_INFO(this->get_logger(), "Started without DEBUG");

    RCLCPP_DEBUG(this->get_logger(), " Publisher node started ");
    if (this->get_parameter("publishing_speed").as_int() < 1000)
      RCLCPP_ERROR(this->get_logger(), "The speed of publish is too high!");
    if (this->get_parameter("publishing_speed").as_int() < 500) {
      RCLCPP_FATAL(this->get_logger(),
                   "The speed of publish is extremely high! Quiting.");
    }
  }


private :
    /**
     * @brief timer_callback is a callback function for the publisher
     *
     */
    void
    timer_callback() {

  RCLCPP_INFO(this->get_logger(),
              " Details \n First Name: %s "
              "\nLast Name: %s "
              "\nAge: %d ",
              message_.first_name.c_str(), message_.last_name.c_str(),
              message_.age);
  details_publisher_->publish(message_);
}

void change_details(
    const std::shared_ptr<week10_hw::srv::Details::Request> request,
    std::shared_ptr<week10_hw::srv::Details::Response> response) {
  response->changed_first_name = request->first_name;
  response->changed_last_name = request->last_name;
  response->changed_age = request->age;
  RCLCPP_WARN(this->get_logger(), "Details Changed ");
  message_.first_name = request->first_name;
  message_.last_name = request->last_name;
  message_.age = request->age;
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

rclcpp::Publisher<week10_hw::msg::Details>::SharedPtr details_publisher_;
rclcpp::TimerBase::SharedPtr timer_;
week10_hw::msg::Details message_; // Custom message to publish
rclcpp::Service<week10_hw::srv::Details>::SharedPtr service_; // service pointer
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetailsPublisher>());
  rclcpp::shutdown();
  return 0;
}