#include <HC_SR04.h> 
#include <PID_v1.h> 
#include <motor_control.h> 
#include <Servo.h> 
#include  <MPU6050.h> 
#include <State.h> 
//parameters for PID 
double setpoint = 0; 
double input;  
double output;  
double kp = 2; 
double ki = 0; 
double kd = 0;  
// setting up MPU6050  
#define SDA 4 
#define SCL 5 
MPU6050 accelerometer (SDA, SCL); 
PID controller (&input, &output, &setpoint, kp, ki, kd, DIRECT); 
//setting up servo 
Servo myservo; 
//setting up distance sensor 
#define ECHO_PIN 2 
#define TRIG_PIN 10 
#define ECHO_INT 0 
HC_SR04 distance_sensor (TRIG_PIN, ECHO_PIN, ECHO_INT); 
//initialize state 
State control_state; 
int Servo_right = 25; 
int Servo_left = 130; //otherwise the sensor view is blocked by the wheels 
int Servo_center = 90; 
int Base_speed = 160;
void setup() { 
  motor_setup(); 
//Initalize state 
//  control_state.setLinearState(150); 
//  control_state.setRotationState(0); 
//PID parameters 
  controller.SetOutputLimits(-100,100); 
  controller.SetSampleTime(25); 
  controller.SetMode(1); 
//begins accelerometer readings   
  accelerometer.initialize(); 
  accelerometer.calibrate(); 
//sets up servo 
  myservo.attach(9); 
  myservo.write(Servo_center); 
//sets up servo 
  distance_sensor.begin(); 
  distance_sensor.start(); 
  Serial.begin(9600); 
  Serial.println("setup done");
} 

int getRange(int pos){ 
  myservo.write(pos); 
  delay(300); 
  distance_sensor.start(); 
  return distance_sensor.getRange(); 
} 

//void movement_direction(int rotate, int linear) { 
//  accelerometer.update(); 
//  control_state.setRotationState(rotate);  
//  control_state.setLinearState(linear);  
//  setpoint = control_state.getRotationState(); 
//  input = accelerometer.get_ang_vel('z');     
//  controller.Compute(); 
//  raw_motor_control(control_state.getLinearState() - output, control_state.getLinearState()+ output); 
//
//} 
void getMostFreeSpace(unsigned int min_pos, unsigned int max_pos, unsigned int interval){
    unsigned int optimalDistance;
    unsigned int optimalPosition;
  for (unsigned int i=min_pos; i <= max_pos; i = i + interval) {
   delay(300);
    if (distance_sensor.isFinished()) {
      int currentDistance = distance_sensor.getRange();
      distance_sensor.start();  
      if (currentDistance < optimalDistance) {
        optimalDistance = currentDistance;
        optimalPosition = i;
        Serial.print(optimalPosition); 
        Serial.print (' '); 
        Serial.println(optimalDistance);
      }
    }
        Serial.print(optimalPosition); 
        Serial.print (' '); 
        Serial.println(optimalDistance);
  }
}
void loop(){ 
//  if(distance_sensor.isFinished()) { 
//      int range = distance_sensor.getRange(); 
//      if( range >=30) { 
//        input = accelerometer.get_ang_vel('z');
//        accelerometer.update();
//        //compute PID value
//        controller.Compute();
//        //set values based on output 
//         raw_motor_control(Base_speed - output, Base_speed+ output); 
//      } else { 
//        dwell(); 
//        delay(15); 
//        backward(200); 
//        delay(300); 
//        dwell(); 
//        int rightRange = getRange(Servo_right); 
//        delay(100); 
//        int leftRange = getRange(Servo_left); 
//        delay(100); 
//        if (rightRange > leftRange) { 
//          diff_right(200);  
//          delay(250); 
//          dwell(); 
//        } else { 
//          diff_left(200); 
//          delay(250); 
//          dwell(); 
//        } 
//      } 
//    distance_sensor.start();     
//  } 
 
getMostFreeSpace(45,135,10);
} 
