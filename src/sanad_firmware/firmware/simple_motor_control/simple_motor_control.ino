#include <PID_v1.h>

// ─── BTS7960 MOTOR PINS ──────────────────────────────
#define RPWM1  5
#define LPWM1  6

// ─── ENCODER PINS ────────────────────────────────────
#define RIGHT_ENC_A  3
#define RIGHT_ENC_B  7

// ─── SAFETY ──────────────────────────────────────────
#define MAX_SAFE_PWM     180    // ~70% power, safe for motors
#define MAX_SAFE_SPEED   10000.0  // Max counts/sec you allow as setpoint

// ─── ENCODER VARIABLES ───────────────────────────────
volatile long right_enc_counter = 0; 
// ─── TIMING ──────────────────────────────────────────
unsigned long last_millis = 0;
const unsigned long INTERVAL_MS = 100;

// ─── PID VARIABLES ───────────────────────────────────
double setpoint = 0.0;
double measured = 0.0;
double output   = 0.0;


double Kp = 0.1, Ki = 1.0, Kd = 0.00;

PID motorPID(&measured, &output, &setpoint, Kp, Ki, Kd, DIRECT);

String serial_buf = "";

// ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  stopMotor();

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), encoderISR, RISING);

  // ── KEY FIX: PID output limits match full PWM range ──
  // Motor protection is done in applyMotor(), NOT here
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-255, 255);
  motorPID.SetSampleTime(INTERVAL_MS);

  Serial.println("=== MOTOR PID READY ===");
  Serial.println("Commands:");
  Serial.println("  s <value>  → Set speed in counts/sec (e.g. s 200)");
  Serial.println("  p <value>  → Set Kp (e.g. p 0.5)");
  Serial.println("  i <value>  → Set Ki (e.g. i 0.01)");
  Serial.println("  d <value>  → Set Kd (e.g. d 0.001)");
  Serial.println("  stop       → Stop motor");
  Serial.println("  gains      → Print current PID gains");
  Serial.println("========================");
}

// ──────────────────────────────────────────────────────
void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - last_millis >= INTERVAL_MS) {

    // ── ATOMIC READ ──
    noInterrupts();
    long temp_count = right_enc_counter;
    right_enc_counter = 0;
    interrupts();

    // ── VELOCITY: counts per second ──
    double scale = 1000.0 / (double)INTERVAL_MS;
    measured = (double)(temp_count) * scale;

    // ── PID COMPUTE ──
    motorPID.Compute();

    // ── APPLY WITH HARDWARE SAFETY CAP ──
    applyMotor(output);

    // ── DEBUG OUTPUT ──
    Serial.print("SP:");     Serial.print(setpoint);
    Serial.print(" CV:");    Serial.print(measured);
    Serial.print(" OUT:");   Serial.print(output);
    Serial.print(" PWM:");   
    Serial.println(constrain((int)output, -MAX_SAFE_PWM, MAX_SAFE_PWM));

    last_millis = now;
  }
}

// ──────────────────────────────────────────────────────
// MOTOR DRIVE — safety cap lives HERE, not in PID limits
// ──────────────────────────────────────────────────────
void applyMotor(double value) {
  // PID calculates freely up to ±255
  // We clamp at MAX_SAFE_PWM to protect hardware
  int pwm_val = constrain((int)value, -MAX_SAFE_PWM, MAX_SAFE_PWM);

  if (pwm_val > 0) {
    analogWrite(RPWM1, pwm_val);
    analogWrite(LPWM1, 0);
  }
  else if (pwm_val < 0) {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, abs(pwm_val));
  }
  else {
    stopMotor();
  }
}

void stopMotor() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 0);
}

// ──────────────────────────────────────────────────────
// ENCODER ISR
// ──────────────────────────────────────────────────────
void encoderISR() {
  if (digitalRead(RIGHT_ENC_B) == LOW) {
    right_enc_counter--;
  } else {
    right_enc_counter++;
  }
}

// ──────────────────────────────────────────────────────
// SERIAL INTERFACE
// ──────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_buf.length() > 0) {
        parseCommand(serial_buf);
        serial_buf = "";
      }
    } else {
      serial_buf += c;
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();

  if (cmd == "stop") {
    setpoint = 0;
    stopMotor();
    Serial.println(">> STOPPED");

  } else if (cmd == "gains") {
    Serial.print(">> Kp="); Serial.print(Kp);
    Serial.print(" Ki=");   Serial.print(Ki);
    Serial.print(" Kd=");   Serial.println(Kd);

  } else if (cmd.startsWith("s ")) {
    float requested = cmd.substring(2).toFloat();
    // ── KEY FIX: Clamp setpoint to safe speed range ──
    setpoint = constrain(requested, -MAX_SAFE_SPEED, MAX_SAFE_SPEED);
    Serial.print(">> Setpoint = "); Serial.println(setpoint);

  } else if (cmd.startsWith("p ")) {
    Kp = cmd.substring(2).toFloat();
    motorPID.SetTunings(Kp, Ki, Kd);
    Serial.print(">> Kp = "); Serial.println(Kp);

  } else if (cmd.startsWith("i ")) {
    Ki = cmd.substring(2).toFloat();
    motorPID.SetTunings(Kp, Ki, Kd);
    Serial.print(">> Ki = "); Serial.println(Ki);

  } else if (cmd.startsWith("d ")) {
    Kd = cmd.substring(2).toFloat();
    motorPID.SetTunings(Kp, Ki, Kd);
    Serial.print(">> Kd = "); Serial.println(Kd);

  } else {
    Serial.print(">> Unknown command: "); Serial.println(cmd);
  }
}