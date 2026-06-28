// ===== BTS7960 + Dual Encoder + Serial Control =====

// ================= MOTOR 1 =================
#define RPWM1 5
#define LPWM1 6

// ================= MOTOR 2 =================
#define RPWM2 9
#define LPWM2 10

// ================= ENCODER 1 =================
#define ENCODER1_A 2
#define ENCODER1_B 4

// ================= ENCODER 2 =================
#define ENCODER2_A 3
#define ENCODER2_B 7

#define MAX_PWM 75
volatile long encoder1_count = 0;
volatile long encoder2_count = 0;

// ===== ENCODER PARAMETERS =====
const float encoder_ppr = 600.0;
const float gear_ratio = 129.0 / 40.0;

// quadrature x4 decoding
const float pulses_per_wheel_rev =
  encoder_ppr * 4.0 * gear_ratio;

// ===== SPEED VARIABLES =====
float wheel1_rpm = 0;
float wheel2_rpm = 0;

unsigned long lastTime = 0;
const int sampleTime = 100; // ms

// ===== MOTOR SPEED =====
int pwmValue = 50;

// ===== CURRENT COMMAND =====
char command = 'C'; // ✅ Use 'C' (Stop) as default, must match switch cases

void setup() {

  Serial.begin(115200);

  // Motor pins
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);

  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);

  // Encoder pins
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);

  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, CHANGE);

  lastTime = millis();

  Serial.println("==== MOTOR CONTROL READY ====");
  Serial.println("Commands:");
  Serial.println("W = Forward");
  Serial.println("S = Backward");
  Serial.println("A = Left");
  Serial.println("D = Right");
  Serial.println("C = Stop");
  Serial.println("0-255 = PWM Speed");
}

void loop() {

  // ===== SERIAL COMMANDS =====
  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');
    input.trim();

    // ===== PWM VALUE =====
    if (input.toInt() > 0 || input == "0") {

      pwmValue = constrain(input.toInt(), 0, MAX_PWM);

      Serial.print("PWM set to: ");
      Serial.println(pwmValue);
    }

    // ===== MOVEMENT COMMAND =====
    else if (input.length() > 0) {

      command = toupper(input.charAt(0)); // ✅ Converts to uppercase

      Serial.print("Command: ");
      Serial.println(command);
    }
  }

  // ===== EXECUTE MOVEMENT =====
  switch (command) {

    case 'W': // ✅ Uppercase to match toupper()
      motor1Forward(pwmValue);
      motor2Forward(pwmValue);
      break;

    case 'S': // ✅ Uppercase
      motor1Backward(pwmValue);
      motor2Backward(pwmValue);
      break;

    case 'A': // ✅ Uppercase
      motor1Backward(pwmValue);
      motor2Forward(pwmValue);
      break;

    case 'D': // ✅ Uppercase
      motor1Forward(pwmValue);
      motor2Backward(pwmValue);
      break;

    case 'C': // ✅ Uppercase — Stop
      stopMotors();
      break;
  }

  // ===== CALCULATE RPM =====
  if (millis() - lastTime >= sampleTime) {

    noInterrupts();

    long count1 = encoder1_count;
    long count2 = encoder2_count;

    encoder1_count = 0;
    encoder2_count = 0;

    interrupts();

    float dt = sampleTime / 1000.0;

    wheel1_rpm =
      (count1 / pulses_per_wheel_rev) * (60.0 / dt);

    wheel2_rpm =
      (count2 / pulses_per_wheel_rev) * (60.0 / dt);

    // ===== PRINT RPM =====
    Serial.print("RPM1: ");
    Serial.print(wheel1_rpm);

    Serial.print(" | RPM2: ");
    Serial.print(wheel2_rpm);

    Serial.print(" | PWM: ");
    Serial.print(pwmValue);

    Serial.print(" | CMD: ");
    Serial.println(command);

    lastTime = millis();
  }
}

// ================= MOTOR FUNCTIONS =================

void motor1Forward(int pwm) {
  analogWrite(RPWM1, pwm);
  analogWrite(LPWM1, 0);
}

void motor1Backward(int pwm) {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, pwm);
}

void motor2Forward(int pwm) {
  analogWrite(RPWM2, pwm);
  analogWrite(LPWM2, 0);
}

void motor2Backward(int pwm) {
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, pwm);
}

void stopMotors() {

  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 0);

  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 0);
}

// ================= ENCODER ISR =================

void encoder1ISR() {

  bool A = digitalRead(ENCODER1_A);
  bool B = digitalRead(ENCODER1_B);

  if (A == B)
    encoder1_count++;
  else
    encoder1_count--;
}

void encoder2ISR() {

  bool A = digitalRead(ENCODER2_A);
  bool B = digitalRead(ENCODER2_B);

  if (A == B)
    encoder2_count++;
  else
    encoder2_count--;
}