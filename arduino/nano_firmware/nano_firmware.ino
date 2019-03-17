#include <Wire.h>
#define SLAVE_ADDRESS     0x21  //TopTower
//#define SLAVE_ADDRESS     0x22  //BottomTower
//#define SLAVE_ADDRESS     0x23
//#define SLAVE_ADDRESS     0x24
//#define SLAVE_ADDRESS     0x25

#define RECIEVED_SIZE     4
#define SENT_SIZE         4
#define PWM_PIN           3
#define DIR_PIN           2
#define POT_PIN           A1
#define ENC_PIN           8

byte recievedSetPoint[RECIEVED_SIZE];
byte sentPosition[SENT_SIZE];

int setPoint = 0;
int currPosition = 0;
//int GAIN = 5;
int GAIN = 270;

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  currPosition = map(analogRead(POT_PIN), 0, 1023, 0,269);
  setPoint = currPosition;
  Serial.begin(9600);
}

void loop() {
  currPosition = map(analogRead(POT_PIN), 0, 1023, 0,269);
  int duty = (GAIN*(setPoint - currPosition))/270;
  if(duty < 0){
    duty = duty*-1;
    digitalWrite(DIR_PIN, HIGH);
  }
  else{
    digitalWrite(DIR_PIN, LOW);
  }
  analogWrite(PWM_PIN, duty);
//    Serial.println(duty);
//    Serial.print("Duty: ");
//    Serial.print(duty);
//    Serial.print(" // Current: ");
//    Serial.print(currPosition);
//    Serial.print(" // Target: ");
//    Serial.println(setPoint);

  Serial.print("Current: ");
  Serial.print(currPosition);
  Serial.print("    Set: ");
  Serial.print(setPoint);
  Serial.print("    Diff: ");
  Serial.print(setPoint - currPosition);
  Serial.print("    Duty: ");
  Serial.print(duty);
  Serial.print("    Gain: ");
  Serial.print(GAIN);
  Serial.print("    GD: ");
  Serial.print(GAIN*(setPoint - currPosition));
  Serial.print("    GDD: ");
  Serial.print((GAIN*(setPoint - currPosition))/360);
  Serial.print("    GDD: ");
  Serial.print(GAIN*(setPoint - currPosition)/360);
  Serial.print("    Dir: ");
  Serial.println(digitalRead(DIR_PIN));

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
      Serial.println(recievedSetPoint[i]);
    }
    else
    {
      Wire.read();
    }
  }
  setPoint = recievedSetPoint[1]<<8 + recievedSetPoint[0];
  Serial.println(setPoint);
  bool state = digitalRead(13);
  digitalWrite(13, !state);
}

void get_byte_position(){
  sentPosition[0] = (currPosition & 255); 
  sentPosition[1] = (currPosition & (255<<8))>>8;
}

void serialEvent() {    
    setPoint = (((Serial.parseInt())%270));
    
}
