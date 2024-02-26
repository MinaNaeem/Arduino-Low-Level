#define en2A 18 // 23
#define en2B 19 // 19

#define en1A 2 // 32
#define en1B 3 // 33

#define Motor1en1 9 //14
#define Motor1en2 8 //12
#define Motor1pwm 10 //13


#define Motor2en1 11 //25
#define Motor2en2 12 //26
#define Motor2pwm 13 //27

#define encoderCounts 1000
#define GearRatio 201 //Change this number

#define Forward 1
#define Reverse -1

volatile int32_t pos1 = 0;
int32_t prevPos1 = 0;
volatile int32_t pos2 = 0;
int32_t prevPos2 = 0;
unsigned long long prevT = 0;

float kp = 0.09;
int ki = 5;

void readEncoder1(){
if(digitalRead(en1B) == LOW) pos1++;
else pos1--;
}

void readEncoder2(){
if(digitalRead(en2B) == LOW) pos2--;
else pos2++;
}

void SetMotor(int dir, int pwr){
if(dir == Forward){
digitalWrite(Motor1en1,HIGH);
digitalWrite(Motor2en1,HIGH);
digitalWrite(Motor1en2,LOW);
digitalWrite(Motor2en2,LOW);
}else if(dir == Reverse)
{
digitalWrite(Motor1en1,LOW);
digitalWrite(Motor2en1,LOW);
digitalWrite(Motor1en2,HIGH);
digitalWrite(Motor2en2,HIGH);

}
analogWrite(Motor1pwm, pwr);
analogWrite(Motor2pwm, pwr);

}

void setup() {
  // put your setup code here, to run once:

pinMode(Motor1en1, OUTPUT);
pinMode(Motor1en2, OUTPUT);
pinMode(Motor1pwm, OUTPUT);
pinMode(Motor2en1, OUTPUT);
pinMode(Motor2en2, OUTPUT);
pinMode(Motor2pwm, OUTPUT);

pinMode(en1A, INPUT_PULLUP);
pinMode(en1B, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(en1A) , readEncoder1, RISING);


pinMode(en2A, INPUT_PULLUP);
pinMode(en2B, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(en2A), readEncoder2, RISING);


Serial.begin(9600);
while(!Serial){}


}

void loop() {
  // put your main code here, to run repeatedly:
unsigned long long currT = micros();
float deltaT = ((float)(currT-prevT))/1.0e6;
float velocity1 = (pos1 - prevPos1)/deltaT;
float velocity2 = (pos2 - prevPos2)/deltaT;
float v1 = velocity1 * 60.0 / GearRatio;
float v2 = velocity2 * 60.0 / GearRatio;

prevPos1 = pos1;
prevPos2 = pos2;
prevT = currT;

// Serial.print(velocity1); //velocity in encoderticks/second
// Serial.println();
// Serial.print(velocity2);
// Serial.println();
// float vavg = (v1+v2)/2;

float vtarget = -200;
float error = vtarget - v2;  //Error of Right Motor
// float error = vtarget - v1;

  float controlsignal = kp*error;

int dir = Reverse;
if(controlsignal<0){
  dir = Forward;
}
int pwr = (int)fabs(controlsignal);
if(pwr > 255) pwr = 255;
SetMotor(dir, pwr);


Serial.println(v1);     //velocity in rpm
Serial.print(",");
Serial.println(vtarget);
// Serial.print(v2);
// Serial.println();
}
