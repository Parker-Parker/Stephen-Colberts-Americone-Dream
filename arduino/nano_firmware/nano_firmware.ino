#include <Wire.h>
#define SLAVE_ADDRESS     0x21  //TopTower
//#define SLAVE_ADDRESS     0x22  //BottomTower
//#define SLAVE_ADDRESS     0x23
//#define SLAVE_ADDRESS     0x24
//#define SLAVE_ADDRESS     0x25

#define RECIEVED_SIZE     4
#define SENT_SIZE         20
//#define SENT_SIZE         4
#define PWM_PIN           5
#define DIR_PIN           6
#define POT_PIN           A1
#define ENC_PIN           8

byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

volatile float setPoint = 0;
float currPosition = 0;
//int GAIN = 5;
int GAIN = 270;

// position control var
float p = 0;
float i = 0;
float d = 0;
int Lower_Bound = 0;
int Upper_Bound = 0;






unsigned long prev_time = 0;
unsigned long curr_time = 0;
float curr_error = 0.0;
float prev_error = 0.0;
float integral_error = 0.0;
float deriv_error = 0.0;
unsigned long dt = 1;

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  currPosition = (map(analogRead(POT_PIN), 0, 1023, 0,807))/3;
  setPoint = currPosition;
  //top_tower
  if (SLAVE_ADDRESS == 0x21){
    p = 7;
    i = 0.003;
    d = 0.01;
    Lower_Bound = 115;
    Upper_Bound = 170;
  }
//bottom tower
  else if (SLAVE_ADDRESS == 0x22){
    p = 3;
    i = 0.004;
    d = 0;
    Lower_Bound = 35;
    Upper_Bound = 105;
  }
  Serial.begin(9600);
  prev_time = millis();


}

void loop() {
  currPosition = (map(analogRead(POT_PIN), 0, 1023, 0,807))/3;
  curr_error = setPoint - currPosition;
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
    duty = duty + 10;
  }

  if(duty > 90){
    duty = 90;
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
  setPoint = (((recievedSetPoint[1])<<8) + recievedSetPoint[0])/100;
  if (setPoint > 360){
    setPoint = (map(analogRead(POT_PIN), 0, 1023, 0,807))/3;
  }
  if(setPoint < Lower_Bound){
    setPoint = Lower_Bound;
  }
  if(setPoint > Upper_Bound){
    setPoint = Upper_Bound;
  }
  bool state = digitalRead(13);
  digitalWrite(13, !state);
}

void get_byte_position(){
  sentPosition[0] = (((int)(currPosition*100)) & 255);
  sentPosition[1] = (((int)(currPosition*100)) & (255<<8))>>8;
}

void serialEvent() {

}
