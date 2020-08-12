#include <HC_SR04.h>

#define ECHO_PIN 2
#define TRIG_PIN 10
#define ECHO_INT 0
#include <motor_control.h>
#include <Servo.h>

Servo myservo;
HC_SR04 distance_sensor (TRIG_PIN, ECHO_PIN, ECHO_INT);

int center = 82;
int right = 25;
int left = 130;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  myservo.attach(9);
  myservo.write(center);
  distance_sensor.begin();
  distance_sensor.start();
}

int getRange(int pos){
  myservo.write(pos);
  delay(300);
  distance_sensor.start();
  while(!distance_sensor.isFinished()){
    Serial.println("READING");
  }
  myservo.write(center);
  return distance_sensor.getRange();
}

void loop() {
  if(distance_sensor.isFinished()) {
    int range = distance_sensor.getRange();
    
    if( range > 100) {
      raw_motor_control( 200,  170);
    } else if(range > 30) { //slow down 
      raw_motor_control( 100, 90);
    } else {
      dwell();
      Serial.println("STOP");
  
      delay(15);
      backward(200);
      delay(300);
      dwell();
  
      int rightRange = getRange(right);
      delay(100);
      int leftRange = getRange(left);
      delay(100);
  
      if (rightRange > leftRange) {
        diff_right(200);
      delay(250);
        dwell();
      } else {
        diff_left(200);
      delay(250);
        dwell();
      }

    }
    distance_sensor.start();
    
  }

}
