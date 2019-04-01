//#define PWM_PIN           5
//#define DIR_PIN           6
//#define POT_PIN           A1
#define ENC_PIN           3
#define ENC2_PIN          2
//#define ENC_PIN           2
//#define ENC2_PIN          3
#define ENC_TICKS         6400



//#define DAC_LSB_PIN0   13
//#define DAC_LSB_PIN1   12
//#define DAC_LSB_PIN2   11
//#define DAC_LSB_PIN3   10
//#define DAC_LSB_PIN4   9
//#define DAC_LSB_PIN5   8
//#define DAC_LSB_PIN6   7
//#define DAC_LSB_PIN7   6
//
//#define DAC_MSB_PIN0   5
//#define DAC_MSB_PIN1   4
//#define DAC_MSB_PIN2   A3
//#define DAC_MSB_PIN3   A2
//#define DAC_MSB_PIN4   A1
////#define DAC_MSB_PIN5   0
////#define DAC_MSB_PIN6   0
////#define DAC_MSB_PIN7   0


#define DAC_LSB_PIN0    7
#define DAC_LSB_PIN1    6
#define DAC_LSB_PIN2    5
#define DAC_LSB_PIN3    4
#define DAC_LSB_PIN4    A3
#define DAC_LSB_PIN5    A2
#define DAC_LSB_PIN6    A1


#define DAC_LSB_PIN7   13
#define DAC_MSB_PIN0   12
#define DAC_MSB_PIN1   11
#define DAC_MSB_PIN2   10
#define DAC_MSB_PIN3   9
#define DAC_MSB_PIN4   8
//#define DAC_MSB_PIN5   0
//#define DAC_MSB_PIN6   0
//#define DAC_MSB_PIN7   0

//#define DAC_LSB_PIN0   A1
//#define DAC_LSB_PIN1   A2
//#define DAC_LSB_PIN2   A3
//#define DAC_LSB_PIN3   4
//#define DAC_LSB_PIN4   5
//#define DAC_LSB_PIN5   6
//#define DAC_LSB_PIN6   7
//
//
//#define DAC_LSB_PIN7   8
//#define DAC_MSB_PIN0   9
//#define DAC_MSB_PIN1   10
//#define DAC_MSB_PIN2   11
//#define DAC_MSB_PIN3   12
//#define DAC_MSB_PIN4   13
////#define DAC_MSB_PIN5   0
////#define DAC_MSB_PIN6   0
////#define DAC_MSB_PIN7   0




volatile int setPoint = 0;
volatile int currPosition = 0;
long GAIN = 2000;

volatile bool enc1 = true;
volatile bool enc2 = true;

volatile bool lastEnc = true;
volatile bool encDir = true;

volatile long ticks = 0;

/////////////////////////////////////
// Method 1
/////////////////////////////////////

void fixTicks(){
  ticks = (ticks+ENC_TICKS*2)%ENC_TICKS;
}

void encISR(){
  enc1 = !enc1;
  if(!enc1 != !enc2) {
    ticks++;
  }
  else{
    ticks--;
  }
  fixTicks();
}

void encISR2(){
  enc2 = !enc2;
  if(!enc1 != !enc2) {
    ticks--;
  }
  else{
    ticks++;
  }
  fixTicks();
}



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

void writeDAC(long value) {

//
//  Serial.print("bits: ");
////  Serial.print( (value >> 0) & 1 , (value >> 1) & 1 , (value >> 2) & 1 , (value >> 3) & 1 , (value >> 4) & 1 , (value >> 5) % 2,(value >> 6) % 2, (value >> 7) & 1 ,(value >> 8) & 1 , (value >> 9) & 1 , (value >> 10)& 1 , (value >> 11)& 1 , (value >> 12)& 1 , (value >> 13)& 1 , (value >> 14)& 1 , (value >> 15)& 1 )
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Serial.print( (value >> 0) & 1 );
//  Serial.print( (value >> 1) & 1 );
//  Serial.print( (value >> 2) & 1 );
//  Serial.print( (value >> 3) & 1 );
//  Serial.print( (value >> 4) & 1 );
//  Serial.print( (value >> 5) & 1 );
//  Serial.print( (value >> 6) & 1 );
//  Serial.print( (value >> 7) & 1 );
//
//  Serial.print( (value >> 8) & 1 );
//  Serial.print( (value >> 9) & 1 );
//  Serial.print( (value >> 10)& 1 );
//  Serial.print( (value >> 11)& 1 );
//  Serial.print( (value >> 12)& 1 );
//  Serial.print( (value >> 13)& 1 );
//  Serial.print( (value >> 14)& 1 );
//  Serial.print( (value >> 15)& 1 );
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  digitalWrite(DAC_LSB_PIN0, (value >> 0) & 1 );
  digitalWrite(DAC_LSB_PIN1, (value >> 1) & 1 );
  digitalWrite(DAC_LSB_PIN2, (value >> 2) & 1 );
  digitalWrite(DAC_LSB_PIN3, (value >> 3) & 1 );
  digitalWrite(DAC_LSB_PIN4, (value >> 4) & 1 );
  digitalWrite(DAC_LSB_PIN5, (value >> 5) & 1 );
  digitalWrite(DAC_LSB_PIN6, (value >> 6) & 1 );
  digitalWrite(DAC_LSB_PIN7, (value >> 7) & 1 );

  digitalWrite(DAC_MSB_PIN0, (value >> 8) & 1 );
  digitalWrite(DAC_MSB_PIN1, (value >> 9) & 1 );
  digitalWrite(DAC_MSB_PIN2, (value >> 10)& 1 );
  digitalWrite(DAC_MSB_PIN3, (value >> 11)& 1 );
  digitalWrite(DAC_MSB_PIN4, (value >> 12)& 1 );
//  digitalWrite(DAC_MSB_PIN5, (value >> 13)& 1 );
//  digitalWrite(DAC_MSB_PIN6, (value >> 14)& 1 );
//  digitalWrite(DAC_MSB_PIN7, (value >> 15)& 1 );

}


void setup() {
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);

 // Serial.begin(9600);

  pinMode(DAC_LSB_PIN0, OUTPUT);
  pinMode(DAC_LSB_PIN1, OUTPUT);
  pinMode(DAC_LSB_PIN2, OUTPUT);
  pinMode(DAC_LSB_PIN3, OUTPUT);
  pinMode(DAC_LSB_PIN4, OUTPUT);
  pinMode(DAC_LSB_PIN5, OUTPUT);
  pinMode(DAC_LSB_PIN6, OUTPUT);
  pinMode(DAC_LSB_PIN7, OUTPUT);

  pinMode(DAC_MSB_PIN0, OUTPUT);
  pinMode(DAC_MSB_PIN1, OUTPUT);
  pinMode(DAC_MSB_PIN2, OUTPUT);
  pinMode(DAC_MSB_PIN3, OUTPUT);
  pinMode(DAC_MSB_PIN4, OUTPUT);
//  pinMode(DAC_MSB_PIN5, OUTPUT);
//  pinMode(DAC_MSB_PIN6, OUTPUT);
//  pinMode(DAC_MSB_PIN7, OUTPUT);


  enc1 = digitalRead(ENC_PIN);
  enc2 = digitalRead(ENC2_PIN);
//  // Test setup
//  pinMode(8, OUTPUT);
//  pinMode(9, OUTPUT);
//
//  bool enc1 = digitalRead(ENC_PIN);
//  bool enc2 = digitalRead(ENC2_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), encISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encISR, CHANGE);
//
//  Wire.begin(SLAVE_ADDRESS);
//  Wire.onRequest(requestEvent);
//  Wire.onReceive(receiveEvent);
//  pinMode(PWM_PIN, OUTPUT);
//  pinMode(DIR_PIN, OUTPUT);
////  currPosition = map(analogRead(POT_PIN), 0, 1023, 0,359);
//  setPoint = 0;
//  Serial.begin(9600);
}


int asdf = 0;
long duty;
void loop() {
  writeDAC(ticks);
//  writeDAC(6400);


//  Serial.print("Ticks: ");
//  Serial.println(ticks);
//  Serial.print("Current: ");
//  Serial.print(currPosition);
//  Serial.print("    Set: ");
//  Serial.println(setPoint);
//  Serial.print("    Diff: ");
//  Serial.print(setPoint - currPosition);
//  Serial.print("    Duty: ");
//  Serial.print(duty);
//  Serial.print("    Gain: ");
//  Serial.print(GAIN);
//  Serial.print("    GD: ");
//  Serial.print(GAIN*(setPoint - currPosition));
//  Serial.print("    GDD: ");
//  Serial.print((GAIN*(setPoint - currPosition))/360);
//  Serial.print("    GDD: ");
//  Serial.print(GAIN*(setPoint - currPosition)/360);
//  Serial.print("    Dir: ");
//  Serial.println(digitalRead(DIR_PIN));
  delay(1);

}
