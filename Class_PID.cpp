// PID  Class 
class PID {
  private:
    float Kp, Ki, Kd;
    float previous_error;
    float integral;
    float dt;

  public:
    PID(float _Kp, float _Ki, float _Kd, float _dt) {
      Kp = _Kp;
      Ki = _Ki;
      Kd = _Kd;
      dt = _dt;
      previous_error = 0.0;
      integral = 0.0;
    }

    float calculate(float setpoint, float measured) {
      float error = setpoint - measured;
      integral += error * dt;
      float derivative = (error - previous_error) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;
      return constrain(output, 0, 255); 
    }
};

float alpha = 0.1; // Smoothing factor
float smoothed_output = 0.0;

// Apply smoothing 
void smoothing(float pid_output) {
  smoothed_output = alpha * pid_output + (1 - alpha) * smoothed_output;
  analogWrite(motor_pin, smoothed_output);
}

const int motor_pin = 9; // PWM pin to control the motor
float setpoint = 100.0; 

// PID object 
PID motorPID(1.0, 0.5, 0.1, 0.1); 

void setup() {
  Serial.begin(9600);
  pinMode(motor_pin, OUTPUT); // Set motor pin 
}

void loop() {
  float measured_speed =  random(90, 110);
  float pid_output = motorPID.calculate(setpoint, measured_speed);
  smoothing(pid_output);
  Serial.print("Measured Speed: ");
  Serial.print(measured_speed);
  Serial.print(" PID Output: ");
  Serial.print(pid_output);
  Serial.print(" Smoothed Output: ");
  Serial.println(smoothed_output);
  delay(100); 
}