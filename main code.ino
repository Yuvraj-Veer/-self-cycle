#define ENA 9
#define IN1 2
#define IN2 7
#include <Wire.h>

const int MPU_ADDR = 0x68;  // Change to 0x69 if needed
int16_t accelerometer_x;

void setup() {
    Serial.begin(9600);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // Additional reset step (optional)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);  // Config register
    Wire.write(0x03);  // Set DLPF to reduce noise
    Wire.endTransmission(true);
}

void loop() {
    // Read accelerometer data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    delay(10);  // Allow sensor update
    Wire.requestFrom(MPU_ADDR, 2, true);
    accelerometer_x = Wire.read() << 8 | Wire.read();

    Serial.print("Accelerometer X: ");
    Serial.println(accelerometer_x);

    // Map accelerometer_x to motor speed (minimum 70 to avoid stopping)
    int speed = map(abs(accelerometer_x), 0, 8000, 70, 255);
    speed = constrain(speed, 70, 255);  // Ensure speed stays in range

    Serial.print("Mapped Speed: ");
    Serial.println(speed);

    if (accelerometer_x < 0) {
        Serial.println("Moving Backward");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        Serial.println("Moving Forward");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    analogWrite(ENA, speed);

    delay(250);
}
