#define L298N_enA 9  // PWM (Motor A)
#define L298N_enB 11 // PWM (Motor B)
#define L298N_in4 8
#define L298N_in3 7
#define L298N_in2 13
#define L298N_in1 12

// Encoders
#define right_encoder_phaseA 3
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2
#define left_encoder_phaseB 4

volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;

String right_encoder_sign = "p";
String left_encoder_sign = "p";

double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;

int speedA = 0;
int speedB = 0;

unsigned long lastTime = 0;

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  Serial.begin(115200);

  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);

  lastTime = millis();
}

void loop() {
  handleSerial();

  // Update every 100 ms
  if (millis() - lastTime >= 100) {
    lastTime = millis();

    // Calculate velocity
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel  = (10 * left_encoder_counter  * (60.0 / 385.0)) * 0.10472;

    // Print both motors
    Serial.print("Right: ");
    Serial.print(right_encoder_sign);
    Serial.print(right_wheel_meas_vel);
    Serial.print(" | Left: ");
    Serial.print(left_encoder_sign);
    Serial.println(left_wheel_meas_vel);

    right_encoder_counter = 0;
    left_encoder_counter = 0;
  }

  // Apply motor speeds
  setMotorA(speedA);
  setMotorB(speedB);
}

// ================= SERIAL =================
void handleSerial() {
  if (Serial.available()) {
    char motor = Serial.read();  // A or B
    int value = Serial.parseInt();

    if (motor == 'A') {
      speedA = constrain(value, -255, 255);
    } 
    else if (motor == 'B') {
      speedB = constrain(value, -255, 255);
    }
  }
}

// ================= MOTOR CONTROL =================
void setMotorA(int speed) {
  if (speed > 0) {
    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
    analogWrite(L298N_enA, speed);
  } else if (speed < 0) {
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, HIGH);
    analogWrite(L298N_enA, -speed);
  } else {
    analogWrite(L298N_enA, 0);
  }
}

void setMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(L298N_in3, HIGH);
    digitalWrite(L298N_in4, LOW);
    analogWrite(L298N_enB, speed);
  } else if (speed < 0) {
    digitalWrite(L298N_in3, LOW);
    digitalWrite(L298N_in4, HIGH);
    analogWrite(L298N_enB, -speed);
  } else {
    analogWrite(L298N_enB, 0);
  }
}

// ================= ENCODERS =================
void rightEncoderCallback() {
  if (digitalRead(right_encoder_phaseB) == HIGH)
    right_encoder_sign = "p";
  else
    right_encoder_sign = "n";

  right_encoder_counter++;
}

void leftEncoderCallback() {
  if (digitalRead(left_encoder_phaseB) == HIGH)
    left_encoder_sign = "p";
  else
    left_encoder_sign = "n";

  left_encoder_counter++;
}