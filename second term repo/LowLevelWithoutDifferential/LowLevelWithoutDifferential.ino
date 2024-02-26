#include <ros.h>
#include <std_msgs/Int32.h>
#include "Servo.h"

ros::NodeHandle nh; 

#define LeftMotorWithEncoder
#define RightMotorWithEncoder

#ifdef LeftMotorWithEncoder 
  #define en1A 2 // 32
  #define en1B 3 // 33
  #define Motor1en1 9 //14
  #define Motor1en2 8 //12
  #define Motor1pwm 10 //13
  #define LeftMotor 1
#endif

#ifdef RightMotorWithEncoder 
  #define en2A 18 // 23
  #define en2B 19 // 19
  #define Motor2en1 11 //25
  #define Motor2en2 12 //26
  #define Motor2pwm 13 //27  
  #define RightMotor 2
#endif 

#define encoderCounts 1000
#define GearRatio 201 

#define Forward 1
#define Reverse -1

#define LeftDefaultangle 80
#define RightDefaultangle 70

#define RservoPin 4 //Change this value
#define LservoPin 16 //Change this value

#define ROSTIMEOUT 200 //adjust this value

float integralerrorL = 0;
float integralerrorR = 0;
int kpL = 0.5;
int kpR = 0.5;

int kiL = 5;
int kiR = 5;


uint32_t vtarget = 0;
int Steerangle = 0;

Servo RsteerServo;
Servo LsteerServo;

volatile uint32_t pos1 = 0;
uint32_t prevPos1 = 0;
volatile uint32_t pos2 = 0;
uint32_t prevPos2 = 0;
unsigned long long prevT = 0;

#ifdef LeftMotorWithEncoder
  void readEncoder1(){
  if(digitalRead(en1B) == LOW) pos1++;
  else pos1--;
  }
#endif

#ifdef RightMotorWithEncoder
  void readEncoder2(){
  if(digitalRead(en2B) == LOW) pos2--;
  else pos2++;
  }
#endif

void Steer( int Steerangle)
{
if(Steerangle > 0)
  {
    RsteerServo.write(RightDefaultangle + Steerangle);
    LsteerServo.write(LeftDefaultangle + Steerangle);
  }else if (Steerangle < 0)
  {
    LsteerServo.write(LeftDefaultangle - abs(Steerangle));
    RsteerServo.write(RightDefaultangle - abs(Steerangle));

  }else
  {
    LsteerServo.write(LeftDefaultangle);
    RsteerServo.write(RightDefaultangle);
  }
}

void SetMotor(int Motor, int dir, int pwr){
  if(Motor == LeftMotor)
  {
  if(dir == Forward){
  digitalWrite(Motor1en1,HIGH);
  digitalWrite(Motor1en2,LOW);
  }
  else if(dir == Reverse)
  {
  digitalWrite(Motor1en1,LOW);
  digitalWrite(Motor1en2,HIGH);
  }
  analogWrite(Motor1pwm, pwr);  
  }
  else{
  if(dir == Forward){
  digitalWrite(Motor2en1,HIGH);
  digitalWrite(Motor2en2,LOW);
  }else if(dir == Reverse)
  {
  digitalWrite(Motor2en1,LOW);
  digitalWrite(Motor2en2,HIGH);
  }
  analogWrite(Motor2pwm, pwr);
  }
  }

std_msgs::Int32 msg;
ros::Publisher pub("encoder_value", &msg);

void velCb( const std_msgs::Int32& velocity_msg){
  if(velocity_msg.data != vtarget) 
    vtarget = velocity_msg.data;
}
ros::Subscriber<std_msgs::Int32> sub1("cmd_vel", &velCb );

void angleCb( const std_msgs::Int32& angle_msg){
  Steerangle = angle_msg.data;
  Steer(Steerangle);
}
ros::Subscriber<std_msgs::Int32> sub2("cmd_angle", &angleCb );


void setup() {
  // put your setup code here, to run once:
#ifdef LeftMotorWithEncoder
  pinMode(Motor1en1, OUTPUT);
  pinMode(Motor1en2, OUTPUT);
  pinMode(Motor1pwm, OUTPUT);
  pinMode(en1A, INPUT_PULLUP);
  pinMode(en1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(en1A) , readEncoder1, RISING);
#endif

#ifdef RightMotorWithEncoder
  pinMode(Motor2en1, OUTPUT);
  pinMode(Motor2en2, OUTPUT);
  pinMode(Motor2pwm, OUTPUT);
  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(en2A), readEncoder2, RISING);
#endif


LsteerServo.attach(LservoPin);
RsteerServo.attach(RservoPin);
steerServo.write(LeftDefaultangle);
steerServo.write(RightDefaultangle);



nh.initNode();

nh.advertise(pub);
nh.subscribe(sub1);
nh.subscribe(sub2);
unsigned long long previousTimer = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
if(millis()- previousTimer > ROSTIMEOUT)
{
  msg.data = vavg;
  pub.publish(&msg);
  nh.spinOnce();
  previousTimer = millis();
}

unsigned long long currT = millis();
float deltaT = ((float)(currT-prevT))/1.0e3;
float velocity1 = (pos1 - prevPos1)/deltaT;
float velocity2 = (pos2 - prevPos2)/deltaT;
float v1 = velocity1 * 60.0 / GearRatio;
float v2 = velocity2 * 60.0 / GearRatio;

prevPos1 = pos1;
prevPos2 = pos2;
prevT = currT;

float vavg = (v1+v2)/2;

float errorL = vtarget - v1;
float errorR = vtarget - v2;

integralerrorL = integralerrorL + errorL*deltaT;
float controlsignalLeft = kpL*errorL + kiL*integralerrorL;
integralerrorR = integralerrorR + errorR*deltaT;
float controlsignalRight = kpR*errorR + kiR*integralerrorR;

int dirL = Forward;
if(controlsignalLeft<0){
  dirL = Reverse;
}
int dirR = Forward;
if(controlsignalRight<0){
  dirL = Reverse;
}

int pwrL = (int)fabs(controlsignalLeft);
int pwrR = (int)fabs(controlsignalRight);
if(pwrL > 255) pwrL = 255;
if(pwrR > 255) pwrR = 255;
#ifdef RightMotorWithEncoder
  SetMotor(RightMotor, dirR, pwrR);
#endif

#ifdef LeftMotorWithEncoder
  SetMotor(LeftMotor, dirL, pwrL);
#endif


}
