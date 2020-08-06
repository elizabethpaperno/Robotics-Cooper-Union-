#include <HC_SR04.h>
#include <PID_v1.h>
#include <motor_control.h>
#include <Servo.h>

#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0

HC_SR04 sensor (TRIG_PIN, ECHO_PIN, ECHO_INT);
Servo actuator; 


double setpoint = 10;
double input; 
double output; 
double kp = 25;
double ki = 0;
double kd = 2; 
PID controller (&input, &output, &setpoint, kp, ki, kd, REVERSE);

void setup() {
  // put your setup code here, to run once:
  motor_setup();

  actuator.attach(9);
  actuator.write (90);
//refers to distance sensor
  sensor.begin();
  sensor.start();
 //Pid parameters
 controller.SetOutputLimits(-255,255);
 controller.SetSampleTime(25);
 controller.SetMode(1);

 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sensor.isFinished())
  {
    input = sensor.getRange();
    sensor.start();
  }
  controller.Compute();
  raw_motor_control(output*1.1, output);
  Serial.print ("output: "); Serial.print(output ); Serial.print(" distance: "); Serial.println(input);
  
}
