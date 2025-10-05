#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>

// WiFi specific includes
#include <WiFi.h>

// ---------------- ROS2 Topic and Node Configuration ----------------
rcl_subscription_t subscriber;
std_msgs__msg__Int8 msg;

rcl_publisher_t publisher;
std_msgs__msg__Int32 pub_msg;
rcl_timer_t timer;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ---------------- Motor Pin Configuration ----------------
// Left motor
#define IN1 14
#define IN2 27

// Right motor
#define IN3 26
#define IN4 25

// ---------------- Ultrasonic Sensor Pin Configuration ----------------
#define TRIG_PIN 18
#define ECHO_PIN 36

// ---------------- Motor Control Functions ----------------
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  // Left motor stop, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  // Right motor stop, left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ---------------- ROS2 Subscription Callback ----------------
// This function is called whenever a message is received on the "cmd_vel" topic.
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;

  switch(msg->data) {
    case 0:
      Serial.println("Received: Stop (0)");
      stopMotors();
      break;
    case 1:
      Serial.println("Received: Forward (1)");
      moveForward();
      break;
    case 2:
      Serial.println("Received: Backward (2)");
      moveBackward();
      break;
    case 3:
      Serial.println("Received: Left (3)");
      turnLeft();
      break;
    case 4:
      Serial.println("Received: Right (4)");
      turnRight();
      break;
    default:
      Serial.print("Received unknown command: ");
      Serial.println(msg->data);
      stopMotors();
  }
}

// ---------------- Ultrasonic Sensor Function ----------------
// Function to read ultrasonic sensor distance in cm with a timeout
long getDistance() {
  // Set a maximum timeout of 50ms (corresponding to about 8.5 meters, which is plenty)
  const long timeout = 50000; // 50,000 microseconds = 50 milliseconds

  // Trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pulse with a specific timeout
  long duration = pulseIn(ECHO_PIN, HIGH, timeout);

  // If duration is 0, it means either no object was detected or the sensor is disconnected.
  // In either case, we can return 0 or a special value like -1 to indicate an error.
  if (duration == 0) {
    return 0; // Or return -1; to signify an error/out of range
  }

  // Calculate distance in centimeters
  return duration * 0.0343 / 2;
}

// ---------------- ROS2 Timer Callback for Publishing ----------------
// This function is called periodically by the timer to publish sensor data.
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.data = getDistance();
    rcl_publish(&publisher, &pub_msg, NULL);
    Serial.print("Published distance (cm): ");
    Serial.println(pub_msg.data);
  }
}

// ---------------- Setup Function ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // --- IMPORTANT: CONFIGURE YOUR WIFI AND AGENT INFO HERE ---
  const char* ssid = "tel";
  const char* password = "01066158787qwert";
  // IP address of the computer running the micro-ROS agent
  const char* agent_ip = "192.168.1.13";
  const uint16_t agent_port = 8888;

  // --- WiFi Connection ---
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the micro-ROS transport to use WiFi
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(2000); // Wait for the transport to be set up

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "esp32_wifi_robot", "", &support);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "cmd_vel"
  );

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sensor_reading"
  );

  // Create a timer to trigger the publisher every 200ms
  const unsigned int timer_period = 200;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period),
    timer_callback
  );

  // Create executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
  
  // ---------------- Motor & Sensor Pins Setup ----------------
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors(); // ensure motors are stopped at start

  Serial.println("Micro-ROS WiFi Motor Node started!");
}

// ---------------- Loop Function ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
