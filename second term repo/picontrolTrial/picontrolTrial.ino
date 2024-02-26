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


volatile int32_t pos1 = 0;
int32_t prevPos1 = 0;
volatile int32_t pos2 = 0;
int32_t prevPos2 = 0;
unsigned long long prevT = 0;

float integralerrorL = 0;
float integralerrorR = 0;


float kpL = 0.5;
float kpR = 0.5;

float kiL = 5;
float kiR = 5;

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

#if defined(RightMotorWithEncoder) || defined(LeftMotorWithEncoder)
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
#endif
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

Serial.begin(9600);
while(!Serial){}

}

void loop() {
  // put your main code here, to run repeatedly:
unsigned long long currT = millis();
float deltaT = ((float)(currT-prevT))/1.0e3;
float velocity1 = (pos1 - prevPos1)/deltaT;
float velocity2 = (pos2 - prevPos2)/deltaT;
float v1 = velocity1 * 60.0 / GearRatio;
float v2 = velocity2 * 60.0 / GearRatio;

prevPos1 = pos1;
prevPos2 = pos2;
prevT = currT;


float vtarget = 100;   //100 rpm and 0 rpm
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
Serial.println(vtarget);
Serial.print(",");
Serial.println(v1);
Serial.print(",");
Serial.println(v2);

}
