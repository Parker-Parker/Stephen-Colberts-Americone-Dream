
#include <Wire.h> 

#define ENC_PIN           3
#define ENC2_PIN          2
#define ENC_TICKS         6400
#define MASTER_ADDRESS     0x30  //nano addr
#define LSB_DAC_ADDR       0x0A  //LSB addr
#define MSB_DAC_ADDR       0x4D  //MSB addr

volatile bool lastEnc = true;
volatile bool encDir = true;
volatile long ticks = 0;
volatile bool enc1 = true;
volatile bool enc2 = true;

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



void fixTicks(){
  noInterrupts();
  ticks = (ticks+ENC_TICKS*2)%ENC_TICKS;
  interrupts();
}

void writeDAC() {
  Serial.println(ticks);
//I may have these backwards, LSB pin0 may be largest bit for LSB
 int LSB_Val = (ticks & (0xF)) + ((ticks & 0x380)>>3);
 LSB_Val = LSB_Val << 3;
//byte LSB_Val = ((ticks & (0x1C0))>>6) + ((ticks & 0x380)>>3);
 int MSB_Val = (((ticks & 0x70)>>4) + ((ticks & 0x1C00)>>7))<<4;
 //byte MSB_Val = (((ticks & 0x1E00)>>9) + ((ticks & 0x1C00)>>7))<<4;
 //data frame is 16 bits, 0000[data][data][00]
 byte MSB_Frame[2];
 MSB_Frame[0] = (MSB_Val & 0x3C0)>>6;
 MSB_Frame[1] = (MSB_Val & 0x3F)<<2;

 byte LSB_Frame[2];
 LSB_Frame[0] = (LSB_Val & 0x3C0)>>6;
 LSB_Frame[1] = (LSB_Val & 0x3F)<<2;
 Wire.beginTransmission(MSB_DAC_ADDR);
 Wire.write(MSB_Frame[0]);
 Wire.write(MSB_Frame[1]);
 Wire.endTransmission();
 Wire.beginTransmission(LSB_DAC_ADDR);
 Wire.write(LSB_Frame[0]);
 Wire.write(LSB_Frame[1]);
 Wire.endTransmission();

////yellow line
//  digitalWrite(DAC_LSB_PIN0, (value >> 0) & 1 );
//  digitalWrite(DAC_LSB_PIN1, (value >> 1) & 1 );
//  digitalWrite(DAC_LSB_PIN2, (value >> 2) & 1 );
//  digitalWrite(DAC_LSB_PIN3, (value >> 3) & 1 );
//  digitalWrite(DAC_LSB_PIN4, (value >> 7) & 1 );
//  digitalWrite(DAC_LSB_PIN5, (value >> 8) & 1 );
//  digitalWrite(DAC_LSB_PIN6, (value >> 9) & 1 );
//
////orangle
//  digitalWrite(DAC_LSB_PIN7, (value >> 4) & 1 );
//  digitalWrite(DAC_MSB_PIN0, (value >> 5) & 1 );
//  digitalWrite(DAC_MSB_PIN1, (value >> 6) & 1 );
//  digitalWrite(DAC_MSB_PIN2, (value >> 10)& 1 );
//  digitalWrite(DAC_MSB_PIN3, (value >> 11)& 1 );
//  digitalWrite(DAC_MSB_PIN4, (value >> 12)& 1 );
}

void setup() {
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);
  Wire.begin(MASTER_ADDRESS);
  enc1 = digitalRead(ENC_PIN);
  enc2 = digitalRead(ENC2_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), encISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encISR, CHANGE);
  Serial.begin(9600);

}

void loop() {
  writeDAC();

}
