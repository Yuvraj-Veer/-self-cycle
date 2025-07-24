#include <Wire.h>

#define ENA 9
#define IN1 2
#define IN2 7

const int MPU_ADDR = 0x68;
float angle = 0.0;
float prevError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

// PID coefficients (tune these)
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.2;

void setup() {
  Serial.begin(19200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Optional DLPF config to reduce noise
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  lastTime = micros();
}

void loop() {
  // Time calculation
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Read accelerometer and gyro
  int16_t accX, accZ, gyroZ;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  accX = Wire.read() << 8 | Wire.read(); // X
  Wire.read(); Wire.read();              // skip Y
  accZ = Wire.read() << 8 | Wire.read(); // Z

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  gyroZ = Wire.read() << 8 | Wire.read();

  // Compute accelerometer angle (sideways tilt)
  float accelAngle = atan2(accX, accZ) * 180.0 / PI;

  // Gyro rate in deg/sec
  float gyroRate = gyroZ / 131.0;

  // Complementary filter
  angle = 0.90 * (angle + gyroRate * dt) + 0.1 * accelAngle;

  // PID control
  float setpoint = 0.0;
  float error = setpoint - angle;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  // Set motor direction
  if (output < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // Set motor speed (minimum 70 to overcome dead zone)
  int motorSpeed = constrain(abs(output), 70, 255);
  analogWrite(ENA, motorSpeed);

  // Debug output
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | Output: ");
  Serial.print(output);
  Serial.print(" | Motor Speed: ");
  Serial.println(motorSpeed);
}
