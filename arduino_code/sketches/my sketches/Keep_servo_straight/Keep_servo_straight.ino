#include <Servo.h>
#include <MPU6050.h>

MPU6050 IMU(4,5); 
Servo servo;
int initialAngle = 90;
int range = 90;
void setup() {
  // put your setup code here, to run once:
  IMU.initialize();
  IMU.calibrate();
  Serial.begin(9600);
  servo.attach(9);
  servo.write(initialAngle);
}

void loop() {
  // put your main code here, to run repeatedly:
  float accelValue = IMU.get_accel('x');
  Serial.println(accelValue * 16384.0);
  int servoStraightPos = initialAngle + accelValue * range;
  IMU.update();
  
  servo.write(servoStraightPos);
}
