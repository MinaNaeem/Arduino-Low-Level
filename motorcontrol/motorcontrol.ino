/* This code is for testing the connections of The 2 motors and their encoders */

#define LeftEncoder      
#define RightEncoder
#define LeftMotor
#define RightMotor

#ifdef LeftEncoder 
  #define en1A 2 // 32
  #define en1B 3 // 33
#endif

#ifdef RightEncoder 
  #define en2A 18 // 23
  #define en2B 19 // 19
#endif 

#ifdef LeftMotor
  #define Motor1en1 8 //14
  #define Motor1en2 9 //12
  #define Motor1pwm 10 //13
#endif

#ifdef RightMotor
  #define Motor2en1 11 //25
  #define Motor2en2 12 //26
  #define Motor2pwm 13 //27
#endif
volatile int rev2  = 0;
#ifdef LeftEncoder
volatile int32_t pos1 = 0;
volatile int rev1 = 0;
void readEncoder1(){
if(digitalRead(en1B) == LOW) pos1++;
else pos1--;
if(pos1 >= 100) {
  rev1 ++;
  pos1 = 0;
  Serial.print("Position of Left Encoder is ");
  Serial.print(rev1);
  Serial.print(", Position of Right Encoder is ");
  Serial.println(rev2);
  }
if(pos1 <= -100) {
  rev1 --;
  pos1 = 0;
  Serial.print("Position of Left Encoder is ");
  Serial.print(rev1);
  Serial.print(", Position of Right Encoder is ");
  Serial.println(rev2);
  }
}
#endif

#ifdef RightEncoder
volatile int32_t pos2 = 0;

void  readEncoder2(){
if(digitalRead(en2B) == LOW) pos2--;
else pos2++;

if(pos2 >= 100) {
  rev2 ++;
  pos2 = 0;
  Serial.print("Position of Left Encoder is ");
  Serial.print(rev1);
  Serial.print(", Position of Right Encoder is ");
  Serial.println(rev2);
  }
  
if(pos2 <= -100) {
  rev2 --;
  pos2 = 0;
  Serial.print("Position of Left Encoder is ");
  Serial.print(rev1);
  Serial.print(", Position of Right Encoder is ");
  Serial.println(rev2);
  }
}
#endif



// const int freq = 500;
// const int motor1Channel = 0;
// const int motor2Channel = 1;

// const int resolution = 8;

void setup() {
  // put your setup code here, to run once:
pinMode(Motor1en1, OUTPUT);
pinMode(Motor1en2, OUTPUT);
pinMode(Motor1pwm, OUTPUT);
pinMode(Motor2en1, OUTPUT);
pinMode(Motor2en2, OUTPUT);
pinMode(Motor2pwm, OUTPUT);

#ifdef LeftEncoder
  pinMode(en1A, INPUT_PULLUP);
  pinMode(en1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(en1A) , readEncoder1, RISING);
#endif

#ifdef RightEncoder
  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(en2A), readEncoder2, RISING);
#endif

Serial.begin(9600);
while(!Serial){}

// ledcSetup(motor1Channel, freq, resolution);
// ledcSetup(motor2Channel, freq, resolution);
// ledcAttachPin(Motor1pwm, motor1Channel);
// ledcAttachPin(Motor2pwm, motor2Channel);

// ledcWrite(motor1Channel, 50);
// ledcWrite(motor2Channel, 50);


analogWrite(Motor1pwm, 100);
analogWrite(Motor2pwm, 100);

}

void loop() 
{
  // put your main code here, to run repeatedly:


digitalWrite(Motor1en1, HIGH);
digitalWrite(Motor2en1, HIGH);
digitalWrite(Motor1en2, LOW);
digitalWrite(Motor2en2, LOW);
delay(4000);

digitalWrite(Motor1en1, LOW);
digitalWrite(Motor2en1, LOW);
digitalWrite(Motor1en2, LOW);
digitalWrite(Motor2en2, LOW);
delay(4000);
// Serial.print("Position of Left Encoder is ");
// Serial.print(rev1);
// Serial.print(", Position of Right Encoder is ");
// Serial.println(rev2);

digitalWrite(Motor1en1, LOW);
digitalWrite(Motor2en1, LOW);
digitalWrite(Motor1en2, HIGH);
digitalWrite(Motor2en2, HIGH);
delay(4000);
// Serial.print("Position of Left Encoder is ");
// Serial.print(rev1);
// Serial.print(", Position of Right Encoder is ");
// Serial.println(rev2);

digitalWrite(Motor1en1, LOW);
digitalWrite(Motor2en1, LOW);
digitalWrite(Motor1en2, LOW);
digitalWrite(Motor2en2, LOW);
delay(4000);

// Serial.print("Position of Left Encoder is ");
// Serial.print(rev1);
// Serial.print(", Position of Right Encoder is ");
// Serial.println(rev2);


}
