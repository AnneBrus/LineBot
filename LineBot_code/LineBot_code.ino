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


Ultrasonic ultrasonic(USTRIG, USECHO);
ros:: NodeHandle nh;

const int leftspeed = 255; 
const int rightspeed = 0; 

void Forward(){
  
  digitalWrite (LEFTREV, LOW); 
  analogWrite (LEFTFWD, leftspeed); 
 
  digitalWrite (RIGHTREV, LOW); 
  analogWrite (RIGHTFWD, rightspeed); 
}

void stop(){
  
  digitalWrite(LEFTEN, LOW);
  digitalWrite(RIGHTEN, LOW);
}

void drive(){
  
  digitalWrite(LEFTEN, HIGH);
  digitalWrite(RIGHTEN, HIGH);
}

void listener(const geometry_msgs::Twist& msg){
  
  float linear_speed = msg.linear.x;
  float angular_speed = msg.angular.z;
 // Serial.print(linear_speed); Serial.println("");
  //Serial.print(angular_speed);
  float speedleft = linear_speed + angular_speed;
  float speedright = linear_speed - angular_speed; 
  if(speedleft>0){
   analogWrite(LEFTFWD, speedleft);
   digitalWrite(LEFTREV, LOW);
  }
  else{
   analogWrite(LEFTREV, -speedleft);
   digitalWrite(LEFTFWD, LOW);
  }  
  if(speedright>0){
   analogWrite(RIGHTFWD, speedright);
   digitalWrite(RIGHTREV, LOW);
  }
  else{
   analogWrite(RIGHTREV, -speedright);
   digitalWrite(RIGHTFWD, LOW);
  }
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &listener);

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
  
  digitalWrite(LEFTEN, HIGH);
  digitalWrite(RIGHTEN, HIGH);
  
  pinMode(USECHO, INPUT);
  pinMode(USTRIG, OUTPUT);
}

void loop(){
  
  nh.spinOnce();
  digitalWrite(USTRIG, LOW);
  delay(1);
  digitalWrite(USTRIG, HIGH);
  delay(1);
  digitalWrite(USTRIG, LOW);
//  long range = ultrasonic.Ranging(INC);
  //Serial.println(range);
  //Serial.println(ultrasonic.Ranging(INC));
  if (ultrasonic.Ranging(INC)<10){
    stop();
  }
  else{
    drive();
  }   
  delay(1);
  
}
