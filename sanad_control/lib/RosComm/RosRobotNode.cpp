#include "RosRobotNode.h"
#include <WiFi.h>

// Initialize static instance pointer
RosRobotNode* RosRobotNode::_instance = nullptr;

RosRobotNode::RosRobotNode(L298N* l298n, UltrasonicSensor* ultrasonic)
  : _l298n(l298n), _ultrasonic(ultrasonic) {
  _instance = this; // Store instance for static callbacks
}

bool RosRobotNode::initWiFi(const char* ssid, const char* password,
                             const char* agentIp, uint16_t agentPort) {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  set_microros_wifi_transports((char*)ssid, (char*)password, 
                               (char*)agentIp, agentPort);
  delay(2000);
  
  return true;
}

bool RosRobotNode::initROS() {
  _allocator = rcl_get_default_allocator();
  rclc_support_init(&_support, 0, NULL, &_allocator);
  
  // Create node
  rclc_node_init_default(&_node, "esp32_wifi_robot", "", &_support);
  
  // Create subscriber
  rclc_subscription_init_default(
    &key_subscriber,
    &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "cmd_vel"
  );
  
  // Create publisher
  rclc_publisher_init_default(
    &ultra_publisher,
    &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ultrasonic_reading"
  );
  
  // Create timer (200ms period)
  const unsigned int timer_period = 200;
  rclc_timer_init_default(
    &_timer,
    &_support,
    RCL_MS_TO_NS(timer_period),
    timerCallback
  );
  
  // Create executor
  rclc_executor_init(&_executor, &_support.context, 2, &_allocator);
  rclc_executor_add_subscription(&_executor, &key_subscriber, &key_msg, 
                                 &subscriptionCallback, ON_NEW_DATA);
  rclc_executor_add_timer(&_executor, &_timer);
  
  Serial.println("Micro-ROS WiFi Motor Node started!");
  return true;
}

void RosRobotNode::spin() {
  rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(100));
}

// Static callback implementations
void RosRobotNode::subscriptionCallback(const void* msgin) {
  if (_instance == nullptr) return;
  
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  
  switch(msg->data) {
    case 0:
      Serial.println("Received: Stop (0)");
      _instance->_l298n->stop();
      break;
    case 1:
      Serial.println("Received: Forward (1)");
      _instance->_l298n->moveForward();
      break;
    case 2:
      Serial.println("Received: Backward (2)");
      _instance->_l298n->moveBackward();
      break;
    case 3:
      Serial.println("Received: Left (3)");
      _instance->_l298n->rotateLeft();
      break;
    case 4:
      Serial.println("Received: Right (4)");
      _instance->_l298n->rotateRight();
      break;
    default:
      Serial.print("Received unknown command: ");
      Serial.println(msg->data);
      _instance->_l298n->stop();
  }
}

void RosRobotNode::timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (_instance == nullptr || timer == NULL) return;
  
  _instance->ultra_msg.data = _instance->_ultrasonic->getDistance();
  rcl_publish(&_instance->ultra_publisher, &_instance->ultra_msg, NULL);
  
  Serial.print("Published distance (cm): ");
  Serial.println(_instance->ultra_msg.data);
}