#include <iostream>
#include <fstream>
#include "rttest/rttest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class LogSubscriber : public rclcpp::Node {
public:
  LogSubscriber() : Node("log_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "logs", 1,
      [this](const std_msgs::msg::Int64::SharedPtr msg) {
        logMessage(msg->data);
      }
    );

    logFile_.open("log.txt", std::ios::app); // Open log file in append mode
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
  std::ofstream logFile_;

  void logMessage(int value) {
    logFile_ << value << std::endl;
    RCLCPP_INFO(get_logger(), "%ld", value);
  }
};

int main(int argc, char **argv) {

    int cpu_id = 0;
  cpu_set_t cpuset;

  const pid_t thread_ID = getpid();
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  sched_setaffinity(thread_ID, sizeof(cpu_set_t), &cpuset);  
  rttest_set_sched_priority(97, SCHED_RR);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LogSubscriber>());
  rclcpp::shutdown();
  return 0;
}
