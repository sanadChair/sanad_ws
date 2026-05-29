  #include <PID_v1.h>

  // ================= MOTOR PINS =================
  #define RPWM1  5
  #define LPWM1  6

  #define RPWM2  10
  #define LPWM2  9

  // ================= ENCODER PINS =================
  #define ENCODER1_A  2
  #define ENCODER1_B  4

  #define ENCODER2_A  3
  #define ENCODER2_B  7

  // ================= ENCODERS =================
  volatile unsigned int right_encoder_counter = 0;
  volatile unsigned int left_encoder_counter = 0;

  String right_wheel_sign = "p";
  String left_wheel_sign = "p";

  unsigned long last_millis = 0;
  const unsigned long interval = 100;

  // ================= SERIAL COMMAND PARSER =================
  bool is_right_wheel_cmd = false;
  bool is_left_wheel_cmd = false;

  bool is_right_wheel_forward = true;
  bool is_left_wheel_forward = true;

  char value[] = "00.00";
  uint8_t value_idx = 0;
  bool is_cmd_complete = false;

  // ================= PID =================

  // Desired velocity (rad/s)
  double right_wheel_cmd_vel = 0.0;
  double left_wheel_cmd_vel  = 0.0;

  // Measured velocity (rad/s)
  double right_wheel_meas_vel = 0.0;
  double left_wheel_meas_vel  = 0.0;

  // PID output (PWM)
  double right_wheel_cmd = 0.0;
  double left_wheel_cmd  = 0.0;

  // PID tuning
  double Kp_r = 40;
  double Ki_r = 120;
  double Kd_r = 0.;

  double Kp_l = 40;
  double Ki_l = 120;
  double Kd_l = 0.0;

  // PID controllers
  PID rightMotor(
    &right_wheel_meas_vel,
    &right_wheel_cmd,
    &right_wheel_cmd_vel,
    Kp_r, Ki_r, Kd_r,
    DIRECT
  );

  PID leftMotor(
    &left_wheel_meas_vel,
    &left_wheel_cmd,
    &left_wheel_cmd_vel,
    Kp_l, Ki_l, Kd_l,
    DIRECT
  );

  // ================= ENCODER CONFIG =================
  // Encoder = 600 PPR
  // GT2 pulley ratio = 129 / 40
  // Effective pulses per wheel revolution:
  const double GEAR_RATIO = 129.0 / 40.0;
  const double PPR = 600.0;
  const double EFFECTIVE_PPR = PPR * GEAR_RATIO;

  // ================= MOTOR FUNCTIONS =================

  void setRightMotor(double pwm, bool forward)
  {
    pwm = constrain(pwm, 0, 255);

    if(forward)
    {
      analogWrite(RPWM1, pwm);
      analogWrite(LPWM1, 0);
    }
    else
    {
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, pwm);
    }
  }

  void setLeftMotor(double pwm, bool forward)
  {
    pwm = constrain(pwm, 0, 255);

    if(forward)
    {
      analogWrite(RPWM2, pwm);
      analogWrite(LPWM2, 0);
    }
    else
    {
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, pwm);
    }
  }

  void setup()
  {
    // ================= MOTOR PINS =================
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);

    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);

    // Stop motors initially
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);

    // ================= PID =================
    rightMotor.SetMode(AUTOMATIC);
    leftMotor.SetMode(AUTOMATIC);

    rightMotor.SetOutputLimits(0, 255);
    leftMotor.SetOutputLimits(0, 255);

    // ================= SERIAL =================
    Serial.begin(115200);

    // ================= ENCODERS =================
    pinMode(ENCODER1_A, INPUT_PULLUP);
    pinMode(ENCODER1_B, INPUT_PULLUP);

    pinMode(ENCODER2_A, INPUT_PULLUP);
    pinMode(ENCODER2_B, INPUT_PULLUP);

    attachInterrupt(
      digitalPinToInterrupt(ENCODER1_A),
      rightEncoderCallback,
      RISING
    );

    attachInterrupt(
      digitalPinToInterrupt(ENCODER2_A),
      leftEncoderCallback,
      RISING
    );
  }

  void loop()
  {
    // ================= SERIAL PARSER =================
    if (Serial.available())
    {
      char chr = Serial.read();

      // Right wheel command
      if(chr == 'r')
      {
        is_right_wheel_cmd = true;
        is_left_wheel_cmd = false;

        value_idx = 0;
        is_cmd_complete = false;
      }

      // Left wheel command
      else if(chr == 'l')
      {
        is_right_wheel_cmd = false;
        is_left_wheel_cmd = true;

        value_idx = 0;
      }

      // Positive direction
      else if(chr == 'p')
      {
        if(is_right_wheel_cmd)
        {
          is_right_wheel_forward = true;
        }
        else if(is_left_wheel_cmd)
        {
          is_left_wheel_forward = true;
        }
      }

      // Negative direction
      else if(chr == 'n')
      {
        if(is_right_wheel_cmd)
        {
          is_right_wheel_forward = false;
        }
        else if(is_left_wheel_cmd)
        {
          is_left_wheel_forward = false;
        }
      }

      // Separator
      else if(chr == ',')
      {
        if(is_right_wheel_cmd)
        {
          right_wheel_cmd_vel = atof(value);
        }
        else if(is_left_wheel_cmd)
        {
          left_wheel_cmd_vel = atof(value);
          is_cmd_complete = true;
        }

        // Reset parser buffer
        value_idx = 0;

        value[0] = '0';
        value[1] = '0';
        value[2] = '.';
        value[3] = '0';
        value[4] = '0';
        value[5] = '\0';
      }

      // Numeric value
      else
      {
        if(value_idx < 5)
        {
          value[value_idx] = chr;
          value_idx++;
        }
      }
    }

    // ================= SPEED UPDATE =================
    unsigned long current_millis = millis();

    if(current_millis - last_millis >= interval)
    {
      // Convert encoder counts to rad/s
      // interval = 100ms => multiply by 10 for counts/sec

      double right_rpm =
        (10.0 * right_encoder_counter * 60.0) / EFFECTIVE_PPR;

      double left_rpm =
        (10.0 * left_encoder_counter * 60.0) / EFFECTIVE_PPR;

      right_wheel_meas_vel = right_rpm * 0.10472;
      left_wheel_meas_vel  = left_rpm  * 0.10472;

      // PID compute
      rightMotor.Compute();
      leftMotor.Compute();

      // Prevent creep at zero command
      if(right_wheel_cmd_vel == 0.0)
      {
        right_wheel_cmd = 0.0;
      }

      if(left_wheel_cmd_vel == 0.0)
      {
        left_wheel_cmd = 0.0;
      }

      // Send encoder data back exactly same format
      String encoder_read =
        "r" + right_wheel_sign + String(right_wheel_meas_vel) +
        ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";

      Serial.println(encoder_read);

      // Reset counters
      right_encoder_counter = 0;
      left_encoder_counter = 0;

      last_millis = current_millis;

      // ================= MOTOR OUTPUT =================
      setRightMotor(
        right_wheel_cmd,
        is_right_wheel_forward
      );

      setLeftMotor(
        left_wheel_cmd,
        is_left_wheel_forward
      );
    }
  }

  // ================= RIGHT ENCODER =================
  void rightEncoderCallback()
  {
    if(digitalRead(ENCODER1_B) == HIGH)
    {
      right_wheel_sign = "n";
    }
    else
    {
      right_wheel_sign = "p";
    }

    right_encoder_counter++;
  }

  // ================= LEFT ENCODER =================
  void leftEncoderCallback()
  {
    if(digitalRead(ENCODER2_B) == HIGH)
    {
      left_wheel_sign = "n";
    }
    else
    {
      left_wheel_sign = "p";
    }

    left_encoder_counter++;
  }