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

#define DAC_LSB_PIN         A2
#define DAC_MSB_PIN         A1


// planar elbow
float p = 10;
float i = 0.003;
float d = 1.5;
int Lower_Bound = 35;
int Upper_Bound = 105;

unsigned long prev_time = 0;
unsigned long curr_time = 0;
float curr_error = 0.0;
float prev_error = 0.0;
float integral_error = 0.0;
float deriv_error = 0.0;
unsigned long dt = 1;


byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

volatile float setPoint = 0;
volatile float currPosition = 0;
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
  long x1 = x1x1 & 1016;
  
  int x2x2 = analogRead(DAC_MSB_PIN);
  long x2 = x2x2 & (1008);

  //yellow
  long Low_MSB = (x1 & 896) >> 7;
  long Low_LSB = (x1 & 120) >> 3;
  //orange
  long High_MSB = (x2 & 896) >> 4;
  long High_LSB = (x2 & 112);

  long MSB = Low_MSB + High_MSB;
  long LSB = Low_LSB + High_LSB;
  
  ticks = LSB + (MSB << 7);
  
  
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
  Serial.begin(9600);
    prev_time = millis();

}

int reportedPos = 0;
int asdf = 0;
long duty;
void loop() {
  readDAC();


  currPosition = (ticks*360)/ENC_TICKS;

  int error = (setPoint - currPosition);
  if (error>180){
    error = error-360;
  }
  else if (error<-180){
    error = error+360;
  }
  else{
    error = error;
  }
  
  curr_error = error;
  curr_time = millis();
  dt = (curr_time - prev_time) + 1;
  integral_error += (dt)*curr_error; //need to prevent overflow
  if(integral_error > 10000){
    integral_error = 10000;
  }
  
  if(integral_error < -10000){
    integral_error = -10000;
  }
  
  deriv_error = (curr_error - prev_error)/(dt);
  int  duty = (p*curr_error + i*integral_error + d*deriv_error);
  prev_time = curr_time;
  
  if(duty < 0){
  duty = duty*-1;
  digitalWrite(DIR_PIN, HIGH);
  }
  else{
  digitalWrite(DIR_PIN, LOW);
  duty = duty;
  }
  
  if(duty > 90){
  duty = 90;
  }
  analogWrite(PWM_PIN, duty);

//  Serial.print("    Current: ");
//  Serial.print(currPosition);
//  Serial.print("  Set: ");
//  Serial.println(setPoint);
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
  setPoint = (recievedSetPoint[1]<<8) + recievedSetPoint[0];
  // set point sent scaled, 36000 = 360.00
  setPoint = setPoint/100;
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

