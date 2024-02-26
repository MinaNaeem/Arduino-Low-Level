/*This code is tested and Working*/
#define LeftEncoder//define your used encoder        

#ifdef LeftEncoder 
  #define enphaseA 2 // 32
  #define enphaseB 3 // 33
#endif

#ifdef RightEncoder 
  #define enphaseA 18 // 23
  #define enphaseB 19 // 19
#endif 
   
volatile int32_t pos = 0;


void readFunction(){
  
if(digitalRead(enphaseB) == LOW) pos++;
else pos--;

// Serial.println("INSIDE THE ISR");
}

void setup() {
  // put your setup code here, to run once:
pinMode(enphaseA, INPUT_PULLUP);
pinMode(enphaseB, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(enphaseA), readFunction, RISING);
Serial.begin(9600);
while(!Serial){}


}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print("Position of Encoder is ");
Serial.println(pos);

delay(30);  //so you can read the serial monitor
}
