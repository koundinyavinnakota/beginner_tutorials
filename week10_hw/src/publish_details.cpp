#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "week10_hw/msg/details.hpp"
#include "week10_hw/srv/details.hpp"
// #include "week10_hw"

using namespace std::chrono_literals;

class DetailsPublisher : public rclcpp::Node
{
public:
  DetailsPublisher(): Node("details_publisher")
  {
    message_.first_name = "Koundinya";
    message_.last_name = "Vinnakota";
    message_.age = 22;
    
    details_publisher_ = this->create_publisher<week10_hw::msg::Details>("publish_details", 10);
    
    service_ = this->create_service<week10_hw::srv::Details>("change_details",std::bind(&DetailsPublisher::change_details,this,std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(500ms, std::bind(&DetailsPublisher::timer_callback, this));
  }

private:
   /**
   * @brief timer_callback is a callback function for the publisher
   *
   */
  void timer_callback() {    

    
    RCLCPP_INFO(this->get_logger(),
    " Details \n First Name: %s " "\nLast Name: %s " "\nAge: %d ",message_.first_name.c_str(), message_.last_name.c_str(), message_.age);
    details_publisher_->publish(message_);
  }

  
  void change_details(const std::shared_ptr<week10_hw::srv::Details::Request> request,
   std::shared_ptr<week10_hw::srv::Details::Response> response){
    response->changed_first_name = request->first_name;
    response->changed_last_name = request->last_name;
    response->changed_age = request->age;
    message_.first_name = request->first_name;
    message_.last_name = request->last_name;
    message_.age = request->age;
    RCLCPP_INFO(this->get_logger(),
    " Change Details Request \n First Name: %s " "\nLast Name: %s " "\nAge: %d ", request->first_name.c_str(), request->last_name.c_str(), request->age);
    RCLCPP_INFO(this->get_logger(),
    " Changed Details Request \n First Name: %s " "\nLast Name: %s " "\nAge: %d ",response->changed_first_name.c_str(), response->changed_last_name.c_str(), response->changed_age);
  }

  rclcpp::Publisher<week10_hw::msg::Details>::SharedPtr details_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  week10_hw::msg::Details message_ ;// Custom message to publish
  rclcpp::Service<week10_hw::srv::Details>::SharedPtr service_;// service pointer
};



int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetailsPublisher>());
  rclcpp::shutdown();
  return 0;
}