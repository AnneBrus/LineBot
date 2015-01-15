#include <TimerOne.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <Ultrasonic.h>
#define LEFTREV 7
#define LEFTEN 24
#define LEFTFWD 6
#define RIGHTREV 3
#define RIGHTEN 25
#define RIGHTFWD 2
#define USECHO  22 
#define USTRIG  23
#define LED 13

class NewHardware : public ArduinoHardware{
  public: NewHardware():ArduinoHardware(&Serial1, 57600){};
};
ros:: NodeHandle_<NewHardware> nh;

Ultrasonic ultrasonic(USTRIG, USECHO);

const int leftspeed = 255; 
const int rightspeed = 0; 
float globalspeedleft;
float globalspeedright;

void stop(){
  //Can't drive forward. It is possible to drive backwards though
  digitalWrite(LEFTFWD, LOW);
  
  if(globalspeedleft<0){ 
    analogWrite(LEFTREV, -globalspeedleft);
    digitalWrite(LEFTFWD, LOW);
  }
  
  digitalWrite(RIGHTFWD, LOW);
  
  if(globalspeedright<0){
    analogWrite(RIGHTREV, -globalspeedright);
    digitalWrite(RIGHTFWD, LOW);
  }
  
  
}

void drive(){
  //If the global speed is higher than 0, drive forward
  //If the global speed is lower than 0, drive backward
  if (globalspeedleft>0){
    analogWrite(LEFTFWD, globalspeedleft);
    digitalWrite(LEFTREV, LOW);
  }
  else{
    analogWrite(LEFTREV, -globalspeedleft);
    digitalWrite(LEFTFWD, LOW);
    
  }
  if (globalspeedright>0){
    analogWrite(RIGHTFWD, globalspeedright);
    digitalWrite(RIGHTREV, LOW);
  }
  else{
    analogWrite(RIGHTREV, -globalspeedright);
    digitalWrite(RIGHTFWD, LOW);
  }
}

void unable(){
  //Disables the wheels
  digitalWrite(LEFTEN, LOW);
  digitalWrite(RIGHTEN, LOW); 
}

void enable(){
  //Enables the wheels
  digitalWrite(LEFTEN, HIGH);
  digitalWrite(RIGHTEN, HIGH);
}

void listener(const geometry_msgs::Twist& msg){
  
  enable();
  //Restart a timer when it receives a signal
  Timer1.restart(); 

  // Motor used to start at ~50 linear.x and stops at 255.
  // Added a multiplier so the motor can be controlled between 10 and 51  
  float linear_speed = 5*msg.linear.x;
  // High sensitivity angular speed
  float angular_speed = 60*msg.angular.z;
  
  // Easy algorithm for driving to the left and right
  // Also making the speeds global
  globalspeedleft = linear_speed + angular_speed;
  globalspeedright = linear_speed - angular_speed; 

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &listener);

void interrupt_callback(){
  unable();
}
void setup(){
  
  nh.initNode();
  nh.subscribe(sub);
 // Serial.begin(9600);
  pinMode(LEFTFWD, OUTPUT);
  pinMode(LEFTEN, OUTPUT); 
  pinMode(LEFTREV, OUTPUT);

  pinMode(RIGHTFWD, OUTPUT);
  pinMode(RIGHTEN, OUTPUT); 
  pinMode(RIGHTREV, OUTPUT);
  
  enable();
  
  pinMode(USECHO, INPUT);
  pinMode(USTRIG, OUTPUT);
  Timer1.initialize(1500000);
  Timer1.attachInterrupt(interrupt_callback); 
}

void loop(){
  
  nh.spinOnce();
  digitalWrite(USTRIG, LOW);
  delay(1);
  digitalWrite(USTRIG, HIGH);
  delay(1);
  digitalWrite(USTRIG, LOW);
  
  if (ultrasonic.Ranging(INC)<10){
    stop();
  }
  else{
    drive();
  }   
  delay(1);
  
}
