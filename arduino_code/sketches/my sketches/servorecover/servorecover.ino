#include <Servo.h>
Servo servo1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo1.attach(9);
  servo1.write(90);
  int x = 6;
  return(x)
  Serial.println(x);
}

void loop() {
  // put your main code here, to run repeatedly:

}
