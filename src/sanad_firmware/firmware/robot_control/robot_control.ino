// ===== BTS7960 + Dual Encoder + PID — SanadInterface Protocol =====
//
// ROS2 sends:    r[p|n]XX.XX,l[p|n]XX.XX,\n
// Arduino sends: r[p|n]XX.XX,l[p|n]XX.XX,\n  (every 100 ms)
//
// Wiring:
//   RPWM1=5  LPWM1=6   → Right motor (BTS7960)
//   RPWM2=9  LPWM2=10  → Left  motor (BTS7960)
//   ENCODER1_A=2(INT0)  ENCODER1_B=4  → Right encoder
//   ENCODER2_A=3(INT1)  ENCODER2_B=7  → Left  encoder
// ===================================================================

#include <PID_v1.h>

// ================= MOTOR PINS =================
#define RPWM1  5
#define LPWM1  6
#define RPWM2  9
#define LPWM2  10

// ================= ENCODER PINS =================
#define ENCODER1_A  2
#define ENCODER1_B  4
#define ENCODER2_A  3
#define ENCODER2_B  7

// ================= ENCODER PARAMETERS =================
const float encoder_ppr    = 600.0;
const float gear_ratio     = 129.0 / 40.0;
const float pulses_per_rev = encoder_ppr * 4.0 * gear_ratio;  // x4 quadrature

// ================= TIMING =================
unsigned long last_millis = 0;
const unsigned long interval = 100;  // ms

// ================= ENCODER STATE =================
volatile long right_encoder_count = 0;
volatile long left_encoder_count  = 0;
String right_wheel_sign = "p";
String left_wheel_sign  = "p";

// ================= SERIAL PARSE STATE =================
bool    is_right_wheel_cmd     = false;
bool    is_left_wheel_cmd      = false;
bool    is_right_wheel_forward = true;
bool    is_left_wheel_forward  = true;
char    value[8]               = "00.00";
uint8_t value_idx              = 0;

// ================= PID =================
double right_wheel_cmd_vel  = 0.0;  // setpoint  rad/s (absolute)
double left_wheel_cmd_vel   = 0.0;
double right_wheel_meas_vel = 0.0;  // measured  rad/s (absolute)
double left_wheel_meas_vel  = 0.0;
double right_wheel_cmd      = 0.0;  // PID output → PWM (-255..255)
double left_wheel_cmd       = 0.0;

double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 10.5, Ki_l = 7.5, Kd_l = 0.1;

PID rightPID(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel,
             Kp_r, Ki_r, Kd_r, DIRECT);
PID leftPID (&left_wheel_meas_vel,  &left_wheel_cmd,  &left_wheel_cmd_vel,
             Kp_l, Ki_l, Kd_l, DIRECT);

#define MIN_SPEED 10.0

double applyMinSpeed(double vel) {
  if (vel == 0.0)           return 0.0;
  if (abs(vel) < MIN_SPEED) return (vel > 0) ? MIN_SPEED : -MIN_SPEED;
  return vel;
}

// ===================================================================
void setup() {
  Serial.begin(115200);  // Must match SanadInterface: BAUD_115200

  pinMode(RPWM1, OUTPUT);  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);  pinMode(LPWM2, OUTPUT);
  analogWrite(RPWM1, 0);   analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0);   analogWrite(LPWM2, 0);

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  // CHANGE for x4 quadrature decoding
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), leftEncoderISR,  CHANGE);

  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(interval);
  leftPID.SetSampleTime(interval);
  rightPID.SetOutputLimits(-255, 255);
  leftPID.SetOutputLimits(-255, 255);

  last_millis = millis();
}

// ===================================================================
void loop() {

  // ===== SERIAL PARSING =====
  // SanadInterface sends: r[p|n]XX.XX,l[p|n]XX.XX,\n
  while (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd  = false;
      value_idx = 0;
      memset(value, 0, sizeof(value));

    } else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd  = true;
      value_idx = 0;
      memset(value, 0, sizeof(value));

    } else if (chr == 'p') {
      // Direction character — set forward flag, don't put into value buffer
      if      (is_right_wheel_cmd) is_right_wheel_forward = true;
      else if (is_left_wheel_cmd)  is_left_wheel_forward  = true;

    } else if (chr == 'n') {
      // Direction character — set reverse flag, don't put into value buffer
      if      (is_right_wheel_cmd) is_right_wheel_forward = false;
      else if (is_left_wheel_cmd)  is_left_wheel_forward  = false;

    } else if (chr == ',') {
      // Commit value — always a positive decimal from SanadInterface
      value[value_idx] = '\0';
      double vel = applyMinSpeed(atof(value));

      if (is_right_wheel_cmd) {
        right_wheel_cmd_vel = vel;
      } else if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = vel;
      }

      value_idx = 0;
      memset(value, 0, sizeof(value));

    } else if (chr != '\n' && chr != '\r') {
      // Digit or decimal point
      if (value_idx < (sizeof(value) - 1))
        value[value_idx++] = chr;
    }
  }

  // ===== CONTROL LOOP (every 100 ms) =====
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {

    // Atomically snapshot and reset encoder counts
    noInterrupts();
    long snap_right = right_encoder_count;
    long snap_left  = left_encoder_count;
    right_encoder_count = 0;
    left_encoder_count  = 0;
    interrupts();

    float dt = interval / 1000.0;

    // Measured speed in rad/s (absolute — direction from sign variable)
    right_wheel_meas_vel = abs(snap_right) / pulses_per_rev * TWO_PI / dt;
    left_wheel_meas_vel  = abs(snap_left)  / pulses_per_rev * TWO_PI / dt;

    // Update sign from encoder direction
    right_wheel_sign = (snap_right >= 0) ? "p" : "n";
    left_wheel_sign  = (snap_left  >= 0) ? "p" : "n";

    // Zero output when command is zero — prevent integral windup
    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel  == 0.0) left_wheel_cmd  = 0.0;

    rightPID.Compute();
    leftPID.Compute();

    // Apply direction from forward flags to signed PWM output
    double r_pwm = is_right_wheel_forward ?  right_wheel_cmd : -right_wheel_cmd;
    double l_pwm = is_left_wheel_forward  ?  left_wheel_cmd  : -left_wheel_cmd;

    setMotorRight(r_pwm);
    setMotorLeft (l_pwm);

    // ===== FEEDBACK TO ROS2 =====
    // SanadInterface reads: r[p|n]XX.XX,l[p|n]XX.XX,\n
    // Use fabs() — sign is carried by sign char only
    float r_abs = right_wheel_meas_vel;
    float l_abs = left_wheel_meas_vel;

    char buf[64];
    snprintf(buf, sizeof(buf),
      "r%s%s%.2f,l%s%s%.2f,",
      right_wheel_sign.c_str(), (r_abs < 10.0 ? "0" : ""), r_abs,
      left_wheel_sign.c_str(),  (l_abs < 10.0 ? "0" : ""), l_abs);
    Serial.println(buf);

    last_millis = current_millis;
  }
}

// ===================================================================
// MOTOR DRIVERS
// ===================================================================
void setMotorRight(double pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm > 0) { analogWrite(RPWM1, (int)pwm);   analogWrite(LPWM1, 0);          }
  else if (pwm < 0) { analogWrite(RPWM1, 0);           analogWrite(LPWM1, (int)-pwm);  }
  else              { analogWrite(RPWM1, 0);            analogWrite(LPWM1, 0);          }
}

void setMotorLeft(double pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm > 0) { analogWrite(RPWM2, (int)pwm);   analogWrite(LPWM2, 0);          }
  else if (pwm < 0) { analogWrite(RPWM2, 0);           analogWrite(LPWM2, (int)-pwm);  }
  else              { analogWrite(RPWM2, 0);            analogWrite(LPWM2, 0);          }
}

// ===================================================================
// ENCODER ISRs — x4 quadrature decoding
// ===================================================================
void rightEncoderISR() {
  bool A = digitalRead(ENCODER1_A);
  bool B = digitalRead(ENCODER1_B);
  if (A == B) right_encoder_count++;
  else        right_encoder_count--;
}

void leftEncoderISR() {
  bool A = digitalRead(ENCODER2_A);
  bool B = digitalRead(ENCODER2_B);
  // Left motor physically mirrored — invert direction
  if (A == B) left_encoder_count--;
  else        left_encoder_count++;
}