#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int8 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ---------------- Motor Pin Configuration ----------------
// Left motor
#define IN1 14
#define IN2 27
#define ENA 26  // PWM pin

// Right motor
#define IN3 25
#define IN4 33
#define ENB 32  // PWM pin

#define MOTOR_SPEED 200   // 0-255 PWM speed (≈80% of max)

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, MOTOR_SPEED);
}

void turnLeft() {
  // Left motor stop, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, MOTOR_SPEED);
}

void turnRight() {
  // Right motor stop, left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, 0);
}

// ---------------- ROS2 Subscription Callback ----------------
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;

  switch(msg->data) {
    case 0:
      Serial.println("Stop (0)");
      stopMotors();
      break;
    case 1:
      Serial.println("Forward (1)");
      moveForward();
      break;
    case 2:
      Serial.println("Backward (2)");
      moveBackward();
      break;
    case 3:
      Serial.println("Left (3)");
      turnLeft();
      break;
    case 4:
      Serial.println("Right (4)");
      turnRight();
      break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(msg->data);
      stopMotors();
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // micro-ROS transport init (Serial)
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "esp32_motor_subscriber", "", &support);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "cmd_vel"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  // ---------------- Motor Pins Setup ----------------
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM channels for motor speed
  ledcSetup(0, 1000, 8); // channel 0, freq=1kHz, resolution=8-bit
  ledcAttachPin(ENA, 0);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(ENB, 1);

  stopMotors();  // ensure motors are stopped at start

  Serial.println("Micro-ROS Motor Subscriber Node started!");
}

// ---------------- Loop ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
