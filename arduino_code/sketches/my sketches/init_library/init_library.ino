#include <HC_SR04.h> 
#define ECHO_PIN 2 
#define TRIG_PIN 10 
#define ECHO_INT 0 
HC_SR04 distance_sensor (TRIG_PIN, ECHO_PIN, ECHO_INT); 

HC_SR04 distance_sensor_setup(){
  distance_sensor.begin();
  distance_sensor.start();
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
