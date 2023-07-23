#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
 
// Range from 0 to 4095  - adjust to define the length of the pulse width
 
#define SERVOMIN  80  // Minimum value
#define SERVOMAX  600  // Maximum value
 
// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER4  4  //Servo Motor 1 on connector 12
 
// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;
 
void setup() {
 
  // Serial monitor setup
  Serial.begin(115200);
  Serial.println("PCA9685 Servo Test");
 
  // Init PCA9685
  pca9685.begin();
  pca9685.setPWMFreq(50);
 
}

void loop() {
  // sweep
    // Move Motor 0 from 0 to 180 degrees
  for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER4, 0, pwm1);
    // Print to serial monitor
    Serial.print("pos = ");
    Serial.println(posDegrees);
    delay(30);
  }
 
  // Move Motor 1 from 180 to 0 degrees
  for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER4, 0, pwm1);
    // Print to serial monitor
    Serial.print("pos = ");
    Serial.println(posDegrees);
    delay(30);
  }
}
