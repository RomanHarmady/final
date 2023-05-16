#include <iostream>
#include <chrono>
#include <memory>
#include <cstdint>

#include "rttest/rttest.h"
#include "rclcpp/rclcpp.hpp"
#include <realtime_tools/realtime_publisher.h>

#include "std_msgs/msg/int64.hpp"

uint16_t pos = 0;
uint16_t mess = 0;


void callback(const std_msgs::msg::Int64::SharedPtr msg) {
 
  RCLCPP_INFO(rclcpp::get_logger("rt_node"), "Received: '%d'", msg->data);
  pos=msg->data; 
}

int main(int argc, char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rt_node");
  auto publisher = node->create_publisher<std_msgs::msg::Int64>("logs", 10);
  auto subscriber = node->create_subscription<std_msgs::msg::Int64>(
    "encoder_publisher", 10, callback);

  std::chrono::milliseconds loop_rate_ms(2);  // 50 Hz
  std::chrono::high_resolution_clock::time_point startTime, endTime;

  //rt thread
  int cpu_id = 0;
  cpu_set_t cpuset;

  const pid_t thread_ID = getpid();
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  sched_setaffinity(thread_ID, sizeof(cpu_set_t), &cpuset);  
  rttest_set_sched_priority(98, SCHED_RR);
  



  std_msgs::msg::Int64 message;
  while (rclcpp::ok()) {

    startTime = std::chrono::high_resolution_clock::now();
    
    if (pos>3010){
      
      mess = pos*10;
      
    }
    else if (pos<2950){
      
      mess = -pos*10;
      
    }
    else mess = 0;



    
     
     

     publisher->publish(message);
     rclcpp::spin_some(node);
     endTime = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
     message.data = duration.count();
     
     
  }

  rclcpp::shutdown();
  return 0;
}

//--------- get unix time
    //auto currentTime = std::chrono::system_clock::now().time_since_epoch();
    //auto unixTime = std::chrono::duration_cast<std::chrono::microseconds>(currentTime).count();

//-------- between 2 points in code 
     //startTime = std::chrono::high_resolution_clock::now();
     //endTime = std::chrono::high_resolution_clock::now();
     //auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);