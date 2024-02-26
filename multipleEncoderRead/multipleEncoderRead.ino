/* Using incremental optical encoder of Omron with 1000 pulse/rev with open collector output */

//define the used encoder/s
#define RightEncoder
#define LeftEncoder

#ifdef LeftEncoder
  #define en1A 2
  #define en1B 3
#endif

#ifdef RightEncoder
  #define en2A 18
  #define en2B 19
#endif 
   
volatile int32_t pos1 = 0;
volatile int32_t pos2 = 0;


void readEncoder1(){
if(digitalRead(en1B) == LOW) pos1++;
else pos1--;
}


void readEncoder2(){
if(digitalRead(en2B) == LOW) pos2++;
else pos2--;
}

void setup() {
  // put your setup code here, to run once:
#ifdef LeftEncoder
  pinMode(en1A, INPUT_PULLUP);
  pinMode(en1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(en1A), readEncoder1, RISING);
#endif

#ifdef RightEncoder
  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(en2A), readEncoder2, RISING);
#endif
  
Serial.begin(9600);
while(!Serial){}

}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print("Position of LeftEncoder is ");
Serial.print(pos1);

Serial.print(", Position of RightEncoder is ");
Serial.println(pos2);

delay(30);   //so you can read the serial monitor
}
