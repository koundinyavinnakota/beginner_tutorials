/**
 * @file ros_test.cpp
 * @author Koundinya Vinnakota
 * @brief Testing the publisher and subscriber.
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "week11_hw/msg/details.hpp"
#include "week11_hw/srv/details.hpp"

namespace integration_test {

/**
 * @brief Testing fixture to test the client.
 *
 */
class TestingFixture : public testing::Test {
 public:
  /**
   * @brief Construct a new Testing Fixture object.
   *
   */
  TestingFixture() : node_(std::make_shared<rclcpp::Node>("Initial_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Created the node");
  }

  /**
   * @brief Set the Up object.
   *
   */
  void SetUp() override {
    client_ = node_->create_client<week11_hw::srv::Details>("change_details");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Setup Complete");
  }

  /**
   * @brief Send a service to the publisher node.
   *
   * @return true, if the service call is succesfull.
   */
  bool send_service_call() {
    auto request = std::make_shared<week11_hw::srv::Details::Request>();
    request->first_name =
        "Koundinya";  // The request that will be sent via the service.
    request->last_name = "Vinnakota";
    request->age = 22;
    // Wait for services to load.
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "service not available, waiting again...");
    }

    // Sending the service request.
    auto result = client_->async_send_request(request);

    // Waiting for the service to be completed.
    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result.get();

      std::cout << "This happened" << std::endl;
      RCLCPP_INFO(node_->get_logger(),
                  " Changed Details Request \n First Name: %s "
                  "\nLast Name: %s "
                  "\nAge: %d ",
                  response->changed_first_name.c_str(),
                  response->changed_last_name.c_str(), response->changed_age);

      if (response->changed_first_name.c_str() == request->first_name) {
        return true;
      }

      return false;
    }

    // When the service has failed.
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to call service change_publisher_string");

    return "NOK";
  }

  /**
   * @brief The shutdown sequence.
   *
   */
  void TearDown() override { std::cout << "DONE WITH TEARDOWN" << std::endl; }

 protected:
  rclcpp::Node::SharedPtr node_;  //!< The pointer to the rclcpp node.
  rclcpp::Service<week11_hw::srv::Details>::SharedPtr
      service_;                      // service pointer
  week11_hw::msg::Details message_;  // Custom message to publish
  rclcpp::Client<week11_hw::srv::Details>::SharedPtr client_;
};

/**
 * @brief Sanity check for the TestingFixture.
 *
 */
TEST_F(TestingFixture, BasicTest) {
  std::cout << "The actual test is happenning here!" << std::endl;
  EXPECT_TRUE(true);
}

/**
 * @brief Testing the respose of the client service.
 *
 */
TEST_F(TestingFixture, ServiceCallCheck) {
  std::cout << "Service call check" << std::endl;
  EXPECT_TRUE(send_service_call());
}
}  // namespace integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  std::cout << "Testing Complete" << std::endl;

  return tests;
}