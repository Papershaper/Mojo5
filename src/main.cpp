#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
 
// Range from 0 to 4095  - adjust to define the length of the pulse width
 
#define SERVOMIN  100  // Minimum value
#define SERVOMAX  550  // Maximum value
 
// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER4  4  //Servo Motor 1 on connector 12
 
// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;
 
void setup() {
 
  // Serial monitor setup
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("PCA9685 Servo Test");
 
  // Init PCA9685
  pca9685.begin();
  pca9685.setPWMFreq(50);
 
}

void loop() {
  // sweep
  Serial.println("<===== START MIN =====>");
  delay(1000);
    
  // Initialize positions for Servo 1 and Servo 2
  int s1posDegrees = 70;
  int s2posDegrees = 110;
  
  // Sweep from s1's minimum to s2's maximum
  for (; s1posDegrees <= 110 && s2posDegrees >= 70; s1posDegrees++, s2posDegrees--) {
    // Determine PWM pulse width for Servo 1 and Servo 2
    int pwm0 = map(s1posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    int pwm1 = map(s2posDegrees, 0, 180, SERVOMIN, SERVOMAX);

    // Write to PCA9685 for Servo 1 and Servo 2
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER4, 0, pwm1);

    // Print to serial monitor
    Serial.printf("Servo 1 pos = %d, Servo 2 pos = %d\n", s1posDegrees, s2posDegrees);
    delay(30);
  }

  Serial.println("<===== Stop Max =====>");
  delay(1000);

  // Sweep from s1's maximum to s2's minimum
  for (; s1posDegrees >= 70 && s2posDegrees <= 110; s1posDegrees--, s2posDegrees++) {
    // Determine PWM pulse width for Servo 1 and Servo 2
    int pwm0 = map(s1posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    int pwm1 = map(s2posDegrees, 0, 180, SERVOMIN, SERVOMAX);

    // Write to PCA9685 for Servo 1 and Servo 2
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER4, 0, pwm1);

    // Print to serial monitor
    Serial.printf("Servo 1 pos = %d, Servo 2 pos = %d\n", s1posDegrees, s2posDegrees);
    delay(30);
  }
}
