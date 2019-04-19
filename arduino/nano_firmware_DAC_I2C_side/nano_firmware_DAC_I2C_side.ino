#include <Wire.h>
//#define SLAVE_ADDRESS     0x21  //TopTower
//#define SLAVE_ADDRESS     0x22  //BottomTower
//#define SLAVE_ADDRESS     0x23  //base motor furthest
#define SLAVE_ADDRESS     0x24  //base motor elbow
//#define SLAVE_ADDRESS     0x25  //base motor stationary

#define RECIEVED_SIZE     4
#define SENT_SIZE         20
//#define SENT_SIZE         4
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

#define DAC_LSB_PIN         A1
#define DAC_MSB_PIN         A2


// planar elbow
float p = 0.2;
float i = 0.00015;
float d = 0.001;
int Lower_Bound = 35;
int Upper_Bound = 105;
long prev_time = 0;
long curr_time = 0;
int curr_error = 0.0;
int prev_error = 0.0;
float integral_error = 0.0;
float deriv_error = 0.0;
long dt = 1;


byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

volatile long setPoint = 0;
volatile long currPosition = 0;
long GAIN = 7;

volatile bool enc1 = true;
volatile bool enc2 = true;

volatile bool lastEnc = true;
volatile bool encDir = true;

volatile long ticks = 0;

void readDAC(){
//  long x1 = (long)analogRead(DAC_LSB_PIN);


  int x1x1 = analogRead(DAC_LSB_PIN);
  //long x1 = (x1x1 & 1016)>>3;
  long x1 = (x1x1 & 0x3F0);
  
  
  int x2x2 = analogRead(DAC_MSB_PIN)+2;
  long x2 = (x2x2 & (0x3F0));

  //yellow
  long Low_MSB = (x1 & 0x380) >> 7;
  long Low_LSB = (x1 & 0x78) >> 3;
//  Serial.print("Low MSB: ");
//  Serial.print(Low_MSB);
  //orange
  long High_MSB = (x2 & 0x380) >> 4;
  long High_LSB = (x2 & 0x70);
//  Serial.print(" HIGH MSB: ");
//  Serial.print(High_MSB);

  long MSB = Low_MSB + High_MSB;
  long LSB = Low_LSB + High_LSB;

  
//  Serial.print(" MSB: ");
//  Serial.print(MSB);
//  Serial.print(" LSB: ");
//  Serial.print(LSB);
//  Serial.print("MSB: ");
//  Serial.print(MSB);
//  Serial.print(" LSB: ");
//  Serial.println(LSB);
  
  ticks = LSB + (MSB << 7);
  
//  Serial.print(" ticks: ");
//  Serial.println(ticks);
//  
}


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
    prev_time = millis();

}

int reportedPos = 0;
int asdf = 0;
long duty;
void loop() {
  readDAC();


  currPosition = ticks;

  int error = (setPoint - currPosition);
  if (error>ENC_TICKS/2){
    error = error-ENC_TICKS;
  }
  else if (error<-ENC_TICKS/2){
    error = error+ENC_TICKS;
  }
  else{
    error = error;
  }
  
  curr_error = error;
  curr_time = millis();
  dt = (curr_time - prev_time) + 1;
//  Serial.print("dt*curr: ");
//  Serial.print(dt*curr_error);
//  Serial.print(" curr: ");
//  Serial.print(curr_error);
//  Serial.print( "dt: ");
//  Serial.print(dt);
//Serial.print(" prev_i err: ");
//Serial.print(integral_error);
  integral_error += (dt)*curr_error; //need to prevent overflow
//Serial.print(" next_i err: ");
//Serial.print(integral_error);
  if(integral_error > 100000){
    integral_error = 100000;
  }
  
  if(integral_error < -100000){
    integral_error = -100000;
  }

  
  deriv_error = (curr_error - prev_error)/(dt);

  
  int  duty = (p*curr_error + i*integral_error - d*deriv_error);
  prev_time = curr_time;
  
  if(duty < 0){
  duty = duty*-1;
  digitalWrite(DIR_PIN, HIGH);
  }
  else{
  digitalWrite(DIR_PIN, LOW);
  duty = duty;
  }
  
  if(duty > 180){
  duty = 180;
  }
  analogWrite(PWM_PIN, duty);

}

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
  setPoint = ((recievedSetPoint[1]<<8) + recievedSetPoint[0])&0xFFFF;
  // set point sent scaled, 36000 = 360.00
  setPoint = ENC_TICKS*((float)setPoint/36000);
  bool state = digitalRead(13); // um ok
  digitalWrite(13, !state);
}

void get_byte_position(){
  sentPosition[0] = (((int)((float)currPosition *36000/ENC_TICKS))& 255);
  sentPosition[1] = (((int)((float)currPosition *36000/ENC_TICKS))& (255<<8))>>8; // lol clever
}



