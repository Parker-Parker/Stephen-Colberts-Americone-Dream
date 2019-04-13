#include <Wire.h>
//#define SLAVE_ADDRESS     0x21  //TopTower
//#define SLAVE_ADDRESS     0x22  //BottomTower
//#define SLAVE_ADDRESS     0x23  //base motor furthest
//#define SLAVE_ADDRESS     0x24  //base motor elbow
#define SLAVE_ADDRESS     0x25  //base motor stationary

#define RECIEVED_SIZE     4
#define SENT_SIZE         20
//#define SENT_SIZE         4
#define PWM_PIN           5
#define DIR_PIN           6
#define POT_PIN           A1
#define ENC_PIN           3
#define ENC2_PIN          2
//#define ENC_PIN           2
//#define ENC2_PIN          3
#define ENC_TICKS         6400


byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

volatile float setPoint = 0;
volatile float currPosition = 0;
long GAIN = 2000;

volatile bool enc1 = true;
volatile bool enc2 = true;

volatile bool lastEnc = true;
volatile bool encDir = true;

volatile long ticks = 0;


//// bottom stationary
float p = 11;
float i = 0.0008;
float d = 2;

// bottom tower
//float p = 11;
//float i = 0.000;
//float d = 0;

unsigned long prev_time = 0;
unsigned long curr_time = 0;
float curr_error = 0.0;
float prev_error = 0.0;
float integral_error = 0.0;
float deriv_error = 0.0;
unsigned long dt = 1;



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




void setup() {
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);


  enc1 = digitalRead(ENC_PIN);
  enc2 = digitalRead(ENC2_PIN);

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
  //Serial.begin(9600);
  prev_time = millis();

}


int asdf = 0;
long duty;
void loop() {

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
  
  if(duty > 130){
  duty = 130;
  }
  analogWrite(PWM_PIN, duty);


//  Serial.print("Current: ");
//  Serial.print(currPosition);
//  Serial.print("    Set: ");
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
  setPoint = ((recievedSetPoint[1])<<8) + recievedSetPoint[0];
  // setpoints are transmitted scaled, 36000 = 360.00
  setPoint = setPoint/100;
//  Serial.println(setPoint);

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
