#include <HC_SR04.h>
#include <Servo.h>
#define ECHO_PIN 2
#define TRIG_PIN 10
#define ECHO_INT 0

HC_SR04 distance_sensor (TRIG_PIN, ECHO_PIN, ECHO_INT);
Servo myservo;
unsigned int optimalDistance = 0;
unsigned int optimalPosition = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(9); 

  distance_sensor.begin();
  distance_sensor.start();
  
}

int getRange(int pos){ 
  myservo.write(pos); 
  delay(300); 
  distance_sensor.start(); 
  return distance_sensor.getRange(); 
} 
void getMostFreeSpace(unsigned int min_pos, unsigned int max_pos, unsigned int interval){

  for (unsigned int i=min_pos; i <= max_pos; i = i + interval) {
   delay(300);
    if (distance_sensor.isFinished()) {
      int currentDistance = getRange(i);
      distance_sensor.start();  
      if (currentDistance > optimalDistance) {
        optimalDistance = currentDistance;
        optimalPosition = i;
        Serial.print(optimalPosition); 
        Serial.print (' '); 
        Serial.println(optimalDistance);
      }
    }
  }
}

void loop() {
  getMostFreeSpace(45,135,10);
  
}
