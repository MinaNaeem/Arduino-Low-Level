#define LeftEncoder //define your used encoder        
#define RightEncoder
#define LeftMotor
#define RightMotor
  // #define Motor1en1 14 
  // #define Motor1en2 12
  // #define Motor1pwm 13

  // #define Motor2en1 25
  // #define Motor2en2 26
  // #define Motor2pwm 27 
#ifdef LeftMotor
  #define Motor1en1 9 //14
  #define Motor1en2 8 //12
  #define Motor1pwm 10 //13
#endif

#ifdef RightMotor
  #define Motor2en1 11 //25
  #define Motor2en2 12 //26
  #define Motor2pwm 13 //27
#endif

#ifdef LeftEncoder 
  #define en1A 2 // 32
  #define en1B 3 // 33
#endif

#ifdef RightEncoder 
  #define en2A 18 // 23
  #define en2B 19 // 19
#endif 
//-------------------------Macros-----------------------------------------------------
#define TIMESAMPLE 200 //200 milliseconds
#define GearRatio 201 //Change this line to your gear ratio
#define TicksPerRev 1000 //1000 Pulse/Rev
//------------------------------------------------------------------------------------
//------------------------Globals-----------------------------------------------------
volatile uint32_t prevT = 0;
volatile int reading1 = 0;
volatile int reading2 = 0;

double vel1 = 0;
double vel2 = 0;
//------------------------------------------------------------------------------------

void readFunction1(){
if(digitalRead(en1B) == LOW) reading1--;
else reading1++;
}

void readFunction2(){
if(digitalRead(en2B) == LOW) reading2--;
else reading2++;
}

void setup() {
  // put your setup code here, to run once:
pinMode(en1A, INPUT_PULLUP);
pinMode(en1B, INPUT_PULLUP);
pinMode(en2A, INPUT_PULLUP);
pinMode(en2B, INPUT_PULLUP);

pinMode(Motor1en1, OUTPUT);
pinMode(Motor1en2, OUTPUT);
pinMode(Motor1pwm, OUTPUT);
pinMode(Motor2en1, OUTPUT);
pinMode(Motor2en2, OUTPUT);
pinMode(Motor2pwm, OUTPUT);

attachInterrupt(digitalPinToInterrupt(en1A), readFunction1, RISING);
attachInterrupt(digitalPinToInterrupt(en2A), readFunction2, RISING);

Serial.begin(9600);
while(!Serial){}  

analogWrite(Motor1pwm , 255);
analogWrite(Motor2pwm, 255);
digitalWrite(Motor1en1, HIGH);
digitalWrite(Motor1en2, LOW);
digitalWrite(Motor2en1, HIGH);
digitalWrite(Motor2en2, LOW);

Serial.println("Program Started Successfully!");
prevT = millis();
}

void loop() {
if(millis() - prevT >= TIMESAMPLE)
{
vel1 = reading1 / (TIMESAMPLE * 1e-3);
vel2 = reading2 / (TIMESAMPLE * 1e-3);

Serial.print("Velocity1 of Encoder1 is ");
Serial.print(vel1);
Serial.print(" encoderticks/s,\t");
Serial.print("Velocity of Encoder2 is ");
Serial.print(vel2);
Serial.print(" encoderticks/s,\t");

Serial.print((vel1/GearRatio) * 60.0);
Serial.print(" Motor1 Rev per Minute,\t");
Serial.print((vel2/GearRatio) * 60.0);
Serial.println(" Motor2 Rev per Minute,\t");

reading1 = 0;
reading2 = 0;
prevT = millis();
}
}
