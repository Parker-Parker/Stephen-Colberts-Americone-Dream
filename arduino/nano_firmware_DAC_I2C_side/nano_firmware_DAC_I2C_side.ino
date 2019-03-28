#include <Wire.h>
//#define SLAVE_ADDRESS     0x21  //TopTower
//#define SLAVE_ADDRESS     0x22  //BottomTower
//#define SLAVE_ADDRESS     0x23  //base motor furthest
#define SLAVE_ADDRESS     0x24  //base motor elbow
//#define SLAVE_ADDRESS     0x25  //base motor stationary

#define RECIEVED_SIZE     4
#define SENT_SIZE         4
#define PWM_PIN           5
#define DIR_PIN           6
//#define POT_PIN           A1
//////////////  0x24  //////////
//#define ENC_PIN           3
//#define ENC2_PIN          2

////////////  0x23  //////////
//#define ENC_PIN           2
//#define ENC2_PIN          3
#define ENC_TICKS         6400

#define DAC_LSB_PIN         A2
#define DAC_MSB_PIN         A1




byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

volatile int setPoint = 0;
volatile int currPosition = 0;
long GAIN = 20;

volatile bool enc1 = true;
volatile bool enc2 = true;

volatile bool lastEnc = true;
volatile bool encDir = true;

volatile long ticks = 0;

void readDAC(){
//  long x1 = (long)analogRead(DAC_LSB_PIN);


  int x1x1 = analogRead(DAC_LSB_PIN);
  long x1 = map(x1x1, 0, 1023, 0, 127);
  
  int x2x2 = analogRead(DAC_MSB_PIN);
  long x2 = map(x2x2, 0, 1023, 0, 63)<<7;
  
  long yeet = map(x1+x2,0,6655,0,6399);
//  long yeet = map(x1+x2,0,6505,0,6399);
  ticks = yeet;

  
//  Serial.print("LSB: ");
//  Serial.print(x1);
//  Serial.print(" MSB: ");
//  Serial.print(x2);

  
  Serial.print(" AL: ");
  Serial.print(x1x1);
  Serial.print(" AM: ");
  Serial.print(x2x2);
  
  
}

/////////////////////////////////////
// Method 1
/////////////////////////////////////

//void fixTicks(){
//  ticks = (ticks+ENC_TICKS*2)%ENC_TICKS;
//}
//
//void encISR(){
//  enc1 = !enc1;
//  if(!enc1 != !enc2) {
//    ticks++;
//  }
//  else{
//    ticks--;
//  }
//  fixTicks();
//}
//
//void encISR2(){
//  enc2 = !enc2;
//  if(!enc1 != !enc2) {
//    ticks--;
//  }
//  else{
//    ticks++;
//  }
//  fixTicks();
//}



/////////////////////////////////////
// Method 2
/////////////////////////////////////

//
//void fixTicks(){
//  ticks = (ticks+ENC_TICKS)%ENC_TICKS;
//}
//
//void encISR(){
//  encDir = lastEnc? encDir : !encDir;
//  encDir ? ticks++ : ticks--;
//  fixTicks();
//  lastEnc= false;
//}
//
//void encISR2(){
//  encDir = !lastEnc? encDir : !encDir;
//  encDir ? ticks++ : ticks--;
//  fixTicks();
//  lastEnc= true;
//}
//


void setup() {
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
//  pinMode(12, OUTPUT);
//  digitalWrite(12, HIGH);
  
  pinMode(DAC_LSB_PIN, INPUT);
  pinMode(DAC_MSB_PIN, INPUT);
  
//  currPosition = map(analogRead(POT_PIN), 0, 1023, 0,359);
  setPoint = 0;
  Serial.begin(9600);
}

int reportedPos = 0;
int asdf = 0;
long duty;
void loop() {
  readDAC();
//  //////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////
//  currPosition = (ticks*360)/ENC_TICKS;
//  long duty = (GAIN*(setPoint - currPosition))/360;
//  if(duty < 0){
//    duty = duty*-1;
//    digitalWrite(DIR_PIN, HIGH);
//  }
//  else{
//    digitalWrite(DIR_PIN, LOW);
//  }
//  if(duty > 70){
//    duty = 70;
//  }
  //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////
  
  currPosition = (ticks*360)/ENC_TICKS;
//
////  
//////  //start average code
//////  reportedPos = (ticks*360)/ENC_TICKS;
//////  
//////  int p = currPosition - reportedPos;  
//////  while(p<0){
//////    p+=(360);
//////  }
//////  p = p%360;
//////
//////  if(p>180){
//////    if(currPosition>reportedPos){
//////      currPosition = ((currPosition*4 +reportedPos)/5+180)%360;  
//////    }
//////    else {
//////      currPosition = (currPosition*4 +reportedPos)/5;
//////    }
//////  }
//////  
//////  if(p<180){
//////    if(currPosition>reportedPos){
//////      currPosition = (currPosition*4 +reportedPos)/5;
//////    }
//////    else {    
//////      currPosition = ((currPosition*4 +reportedPos)/5+180)%360;  
//////      }
//////  }
//////  //end average code
////
//
  
  int error = (setPoint - currPosition);
  if (error>180){
    duty = (GAIN*(error-360))/180;
  }
  else if (error<-180){
    duty = (GAIN*(error+360))/180;
  }
  else{
    duty = (GAIN*(error))/180;
  }
  
  if(duty < 0){
    duty = duty*-1;
    digitalWrite(DIR_PIN, HIGH);
  }
  else{
    digitalWrite(DIR_PIN, LOW);
  }
  if(duty > 70){
    duty = 70;
  }

  
  analogWrite(PWM_PIN, duty);

  //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////

//  
  Serial.print("    TIC: ");
  Serial.print(ticks);
  Serial.print("    Current: ");
  Serial.print(currPosition);
  Serial.print("  Set: ");
  Serial.print(setPoint);
  Serial.print("      Diff: ");
  Serial.print(setPoint - currPosition);
  Serial.print("     Duty: ");
  Serial.print(duty);
//  Serial.print("  Gain: ");
//  Serial.print(GAIN);
//  Serial.print("    GD: ");
//  Serial.print(GAIN*(setPoint - currPosition));
//  Serial.print("  GDD: ");
//  Serial.print((GAIN*(setPoint - currPosition))/360);
//  Serial.print("    GDD: ");
//  Serial.print(GAIN*(setPoint - currPosition)/360);
  Serial.print("  Dir: ");
  Serial.println(digitalRead(DIR_PIN));

/////////////////////////////////////
// test code
/////////////////////////////////////

//  if(asdf>20){
//  digitalWrite(8, LOW);
//  digitalWrite(9, LOW);
//  digitalWrite(8, HIGH);
//  digitalWrite(9, HIGH);
//  
//  }else{
//  digitalWrite(8, LOW);
//  digitalWrite(9, HIGH);
//  digitalWrite(8, HIGH);
//  digitalWrite(9, LOW);
//  }
//  asdf++;
//  asdf = asdf>40 ? 0 : asdf;
//
//  
//  Serial.print("Deg:  ");
//  Serial.println(currPosition);
//  Serial.print("Tic:  ");
//  Serial.println(ticks);
//  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
//  delay(200);


}
/*
 * 
 */
void requestEvent(){
  get_byte_position();
  Wire.write(sentPosition, SENT_SIZE);
}

void receiveEvent(int bytesReceived){
  for(int i = 0; i < bytesReceived; i++)
  {
    if(i < RECIEVED_SIZE)
    {
      recievedSetPoint[i] = Wire.read();
    }
    else
    {
      Wire.read();
    }
  }
  setPoint = recievedSetPoint[1]<<8 + recievedSetPoint[0];
  Serial.println(setPoint);
  bool state = digitalRead(13); // um ok
  digitalWrite(13, !state);
}

void get_byte_position(){
  sentPosition[0] = (currPosition & 255); 
  sentPosition[1] = (currPosition & (255<<8))>>8; // lol clever
}



void serialEvent() {    
    int yeet = Serial.parseInt()+720;
    if(yeet>=0){
      setPoint = ((yeet%360));
    }
}

