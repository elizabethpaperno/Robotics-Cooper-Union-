#include <MPU6050.h>

#define SDA 5
#define SCL 5

MPU6050 sensor (SDA, SCL);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  sensor.initialize();
  sensor.update();
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.update();
  Serial.println(sensor.get_accel('y'));
}
