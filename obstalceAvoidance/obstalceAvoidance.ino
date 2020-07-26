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
PID controller (&input, &output, &setpoint, kp, ki, kd, REVERSE); 
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

void setup() { 
  motor_setup(); 
//Initalize state 
  control_state.setLinearState(150); 
  control_state.setRotationState(0); 
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
} 

int getRange(int pos){ 
  myservo.write(pos); 
  delay(300); 
  distance_sensor.start(); 
  myservo.write(Servo_center); 
  return distance_sensor.getRange(); 
} 

void movement_direction(int rotate, int linear) { 
  accelerometer.update(); 
  control_state.setRotationState(rotate);  
  control_state.setLinearState(linear);  
  setpoint = control_state.getRotationState(); 
  input = accelerometer.get_ang_vel('z');     
  controller.Compute(); 
  raw_motor_control(control_state.getLinearState() - output, control_state.getLinearState()+ output); 

} 

void loop(){ 
  if(distance_sensor.isFinished()) { 
      int range = distance_sensor.getRange(); 
      if( range >=20) { 
         movement_direction(0,150); 
      } else { 
        dwell(); 
        delay(15); 
        backward(200); 
        delay(300); 
        dwell(); 
        int rightRange = getRange(Servo_right); 
        delay(100); 
        int leftRange = getRange(Servo_left); 
        delay(100); 
        if (rightRange > leftRange) { 
          movement_direction(45,80);  
          delay(250); 
          dwell(); 
        } else { 
          movement_direction(-45,80); 
          delay(250); 
          dwell(); 
        } 
      } 
    distance_sensor.start();     
  } 

} 
