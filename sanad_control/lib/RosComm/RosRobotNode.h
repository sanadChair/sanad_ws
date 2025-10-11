#ifndef ROS_ROBOT_NODE_H
#define ROS_ROBOT_NODE_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>

#include "l298n.h"
#include "ultrasonic.h"

class RosRobotNode {
public:
  RosRobotNode(L298N* l298n, UltrasonicSensor* ultrasonic);
  bool initWiFi(const char* ssid, const char* password, 
                const char* agentIp, uint16_t agentPort);
  bool initROS();
  void spin();
  
  // Static callbacks (needed for C-style function pointers)
  static void subscriptionCallback(const void* msgin);
  static void timerCallback(rcl_timer_t* timer, int64_t last_call_time);
  void destroy();

private:
  L298N* _l298n;
  UltrasonicSensor* _ultrasonic;
  
  // keyboard 
  rcl_subscription_t key_subscriber;
  std_msgs__msg__Int8 key_msg;
  
  // ultrasonic
  rcl_publisher_t ultra_publisher;
  std_msgs__msg__Int32 ultra_msg;
  
  // entities
  rcl_timer_t _timer;
  rclc_executor_t _executor;
  rclc_support_t _support;
  rcl_allocator_t _allocator;
  rcl_node_t _node;
  const char *node_name;
  
  static RosRobotNode* _instance; // For static callback access
};

#endif