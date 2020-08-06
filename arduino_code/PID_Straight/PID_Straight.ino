//libraries for motor control, accelerometer, and PID
#include <PID_v1.h>
#include <motor_control.h>
#include <MPU6050.h>
#include <State.h>
//setting variables for PID controller
double setpoint = 0;
double input; 
double output; 
double kp = 2;
double ki = 0;
double kd = 0; 
// setting up MPU6050 
#define SDA 4
#define SCL 5
MPU6050 sensor (SDA, SCL);
// creates variable which stores base power value for motors
int Base_motor_speed = 160;
PID controller (&input, &output, &setpoint, kp, ki, kd, DIRECT);
//State set up
State controller_state;
controller_state.setLinearState(150);
controller_state.setRotationState(0);
void setup() {
  // put your setup code here, to run once:
   motor_setup();
   //PID parameters
   controller.SetOutputLimits(-100,100);
   controller.SetSampleTime(25);
   controller.SetMode(1);
  //begins accelormeter readings  
   sensor.initialize();
   sensor.calibrate();
   Serial.begin(9600);

   
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(sensor.get_ang_vel('z'));
  input = sensor.get_ang_vel('z');
  Serial.print ("input: "); 
  Serial.println(input); 
  sensor.update();
  //compute PID value
  controller.Compute();
  //set values based on output abd print them
  int newBaseMotorSpeed = controller_state.getLinearState();
  raw_motor_control(Base_motor_speed - output, Base_motor_speed + output);
  Serial.print ("output: "); 
  Serial.println(output ); 
}
