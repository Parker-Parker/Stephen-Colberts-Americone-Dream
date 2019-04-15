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
  enc1 = digitalRead(ENC_PIN);
  enc2 = digitalRead(ENC2_PIN);
  if(!enc1 != !enc2) {
    ticks++;
  }
  else{
    ticks--;
  }
  fixTicks();
}

void encISR2(){
  enc1 = digitalRead(ENC_PIN);
  enc2 = digitalRead(ENC2_PIN);
  if(!enc1 != !enc2) {
    ticks--;
  }
  else{
    ticks++;
  }
  fixTicks();
}




void writeDAC(long value) {

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
}


void setup() {
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);


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


  enc1 = digitalRead(ENC_PIN);
  enc2 = digitalRead(ENC2_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), encISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encISR, CHANGE);

}


int asdf = 0;
long duty;
void loop() {
  writeDAC(ticks);
  delay(1);
}

