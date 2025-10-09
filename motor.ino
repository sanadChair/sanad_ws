#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include message types
#include <std_msgs/msg/int8.h>      // For motor commands
#include <std_msgs/msg/float32.h>   // For sensor data (Updated)
#include <std_msgs/msg/u_int8.h>    // For servo angle (Added)

// WiFi specific includes
#include <WiFi.h>

// Servo specific include (Added)
#include <ESP32Servo.h>

// ---------------- ROS2 Topic and Node Configuration ----------------
// Motor Subscriber
rcl_subscription_t motor_subscriber;
std_msgs__msg__Int8 motor_msg;

// Servo Subscriber (Added)
rcl_subscription_t servo_subscriber;
std_msgs__msg__UInt8 servo_msg;

// Sensor Publisher
rcl_publisher_t publisher;
std_msgs__msg__Float32 pub_msg; // (Updated from Int32 to Float32)
rcl_timer_t timer;

// ROS2 Core Objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ---------------- Pin Configuration ----------------
// Left motor
#define IN1 14
#define IN2 27

// Right motor
#define IN3 26
#define IN4 25

// Ultrasonic Sensor
#define TRIG_PIN 18
#define ECHO_PIN 33 // (Updated from 36 to 33)

// Servo Motor (Added)
#define SERVO_PIN 32
Servo my_servo;

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
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ---------------- ROS2 Callbacks ----------------

// Callback for motor commands on "/cmd_vel" topic
void motor_subscription_callback(const void * msgin) {
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
      Serial.print("Received unknown motor command: ");
      Serial.println(msg->data);
      stopMotors();
  }
}

// Callback for servo angle on "/servo_angle" topic (Added)
void servo_subscription_callback(const void * msgin) {
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    // The Servo library's write() function accepts an angle from 0 to 180.
    int angle = msg->data;
    if (angle >= 0 && angle <= 180) {
        my_servo.write(angle);
        Serial.print("Set servo angle to: ");
        Serial.println(angle);
    } else {
        Serial.print("Received invalid servo angle: ");
        Serial.println(angle);
    }
}

// ---------------- Sensor Function ----------------

// Function to read ultrasonic sensor distance in cm (returns float)
float getDistance() {
  const long timeout = 50000; // 50ms timeout

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, timeout);

  if (duration == 0) {
    return 0.0; // Return 0.0 if out of range or error
  }

  // Calculate distance in centimeters (as a float)
  return duration * 0.0343 / 2.0;
}

// ---------------- ROS2 Timer Callback for Publishing ----------------

// This function publishes the sensor data periodically
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.data = getDistance(); // Assigns the float value
    rcl_publish(&publisher, &pub_msg, NULL);
    Serial.print("Published distance (cm): ");
    Serial.println(pub_msg.data);
  }
}

// ---------------- Setup Function ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // --- CONFIGURE YOUR WIFI AND AGENT INFO ---
  const char* ssid = "tel";
  const char* password = "01066158787qwert";
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
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // --- Create Node ---
  rclc_node_init_default(&node, "esp32_wifi_robot", "", &support);

  // --- Create Motor Subscriber ---
  rclc_subscription_init_default(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "cmd_vel"
  );

  // --- Create Servo Subscriber (Added) ---
  rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "servo_angle"
  );

  // --- Create Sensor Publisher ---
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), // (Updated to Float32)
    "sensor_reading"
  );

  // --- Create Timer for Publisher ---
  const unsigned int timer_period = 200; // ms
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period),
    timer_callback
  );

  // --- Create Executor ---
  // Increased from 2 to 3 to handle the new servo subscriber
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_subscription_callback, ON_NEW_DATA); // (Added)
  rclc_executor_add_timer(&executor, &timer);
  
  // --- Hardware Pins Setup ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  my_servo.attach(SERVO_PIN); // Attach servo to its pin (Added)
  my_servo.write(90);         // Initialize servo to a neutral position

  stopMotors(); // Ensure motors are stopped at start

  Serial.println("Micro-ROS WiFi Robot Node started!");
}

// ---------------- Loop Function ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
