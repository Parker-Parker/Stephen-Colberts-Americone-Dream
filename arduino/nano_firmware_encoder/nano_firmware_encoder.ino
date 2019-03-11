#include <Wire.h>
//#define SLAVE_ADDRESS     0x21  //TopTower
//#define SLAVE_ADDRESS     0x22  //BottomTower
//#define SLAVE_ADDRESS     0x23  //base motor furthest
//#define SLAVE_ADDRESS     0x24  //base motor elbow
#define SLAVE_ADDRESS     0x25  //base motor stationary

#define RECIEVED_SIZE     4
#define SENT_SIZE         4
#define PWM_PIN           5
#define DIR_PIN           6
#define POT_PIN           A1
#define ENC_PIN           2
#define ENC2_PIN          3
#define ENC_TICKS         1024


byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

int setPoint = 0;
int currPosition = 0;
int GAIN = 5;

volatile bool enc1 = true;
volatile bool enc2 = true;

volatile bool lastEnc = true;
volatile bool encDir = true;

volatile long ticks = 0;

/////////////////////////////////////
// Method 1
/////////////////////////////////////

//void fixTicks(){
//  ticks = (ticks+ENC_TICKS)%ENC_TICKS;
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


void fixTicks(){
  ticks = (ticks+ENC_TICKS)%ENC_TICKS;
}

void encISR(){
  encDir = lastEnc? encDir : !encDir;
  encDir ? ticks++ : ticks--;
  fixTicks();
  lastEnc= false;
}

void encISR2(){
  encDir = !lastEnc? encDir : !encDir;
  encDir ? ticks++ : ticks--;
  fixTicks();
  lastEnc= true;
}



void setup() {
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);

//  // Test setup
//  pinMode(8, OUTPUT);
//  pinMode(9, OUTPUT);

  bool enc1 = digitalRead(ENC_PIN);
  bool enc2 = digitalRead(ENC2_PIN);
  
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), encISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encISR, CHANGE);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
//  currPosition = map(analogRead(POT_PIN), 0, 1023, 0,359);
  setPoint = 0;
  Serial.begin(9600);
}


int asdf = 0;
void loop() {
  currPosition = (ticks*360)/ENC_TICKS;
  int duty = GAIN*(setPoint - currPosition)/360;
  if(duty < 0){
    duty = duty*-1;
    digitalWrite(DIR_PIN, HIGH);
  }
  else{
    digitalWrite(DIR_PIN, LOW);
  }
  analogWrite(PWM_PIN, duty);

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
