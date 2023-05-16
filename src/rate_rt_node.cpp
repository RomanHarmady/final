#include <iostream>
#include <chrono>
#include <memory>
#include <cstdint>

#include "rttest/rttest.h"
#include "rclcpp/rclcpp.hpp"
#include <realtime_tools/realtime_publisher.h>

#include "std_msgs/msg/int64.hpp"

auto message = std_msgs::msg::Int64();
std::chrono::high_resolution_clock::time_point startTime, endTime;
uint16_t mess = 0;
uint16_t pos = 0;



class MyPublisherNode : public rclcpp::Node
{
public:
  MyPublisherNode() : Node("my_publisher_node")
  {
    publisher_ = create_publisher<std_msgs::msg::Int64>("logs", 10);

    // Set the target publishing rate to 500 Hz (2ms interval)
    rate_limiter_ = std::make_shared<rclcpp::Rate>(500);

    timer_ = create_wall_timer(std::chrono::microseconds(1), [this]() {
      publish_message();
    });
  }

private:
  void publish_message()
  {
    startTime = std::chrono::high_resolution_clock::now();
    
    if (pos>3010){
      
      mess = pos*10;
      
    }
    else if (pos<2950){
      
      mess = -pos*10;
      
    }
    else mess = 0;

  
     endTime = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
     message.data = duration.count();


    publisher_->publish(message);
    rate_limiter_->sleep();
  }

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Rate> rate_limiter_;
};

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyPublisherNode>();
  int cpu_id = 0;
  cpu_set_t cpuset;
  rclcpp::spin(node);
  const pid_t thread_ID = getpid();
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  sched_setaffinity(thread_ID, sizeof(cpu_set_t), &cpuset);  
  rttest_set_sched_priority(98, SCHED_RR);
  rclcpp::shutdown();
  return 0;
}
