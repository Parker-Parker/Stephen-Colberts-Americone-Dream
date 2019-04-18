#include "HX711.h"
#include <Wire.h>
#define SLAVE_ADDRESS 0x50 //NUB
#define RECIEVED_SIZE     4
#define SENT_SIZE         20

// HX711 circuit wiring
const int z2_S = 2;
const int z2_D = 3;
const int y2_S = 4;
const int y2_D = 5;
const int x_S = 6;
const int x_D = 7;
const int z1_S = 8;
const int z1_D = 9;
const int y1_S = 10;
const int y1_D = 11;

const int avg_buffer_size = 5;
int buffer_index = 0;
long avg_buffer[5][avg_buffer_size];

long y1 = 0;
long y2 = 0;
long z1 = 0;
long z2 = 0;
long x = 0;

long y2_final = 0;
long y1_final = 0;
long z2_final = 0;
long z1_final = 0;
long x_final = 0;

byte sentReadings[20];



HX711 z1_cell;
HX711 z2_cell;
HX711 y1_cell;
HX711 y2_cell;
HX711 x_cell;


void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
 // Wire.onReceive(receiveEvent);
  Serial.begin(57600);
  z2_cell.begin(z2_D, z2_S);
  z1_cell.begin(z1_D, z1_S);
  y2_cell.begin(y2_D, y2_S);
  y1_cell.begin(y1_D, y1_S);
  x_cell.begin(x_D, x_S);
  
}

void loop() {

  if (y2_cell.is_ready()) {
    y2 = y2_cell.read();
  }
  if (y1_cell.is_ready()) {
    y1 = y1_cell.read();
  }
//  Serial.print("Y1 Raw: ");
//  Serial.print(y1);
    if (z2_cell.is_ready()) {
    z2 = z2_cell.read();
  }
    if (z1_cell.is_ready()) {
    z1 = z1_cell.read();
  }
    if (x_cell.is_ready()) {
    x = x_cell.read();
  }
  buffer_index++;
  buffer_index = buffer_index%avg_buffer_size;
  avg_buffer[0][buffer_index] = y2;
  avg_buffer[1][buffer_index] = z2;
  avg_buffer[2][buffer_index] = x;
  avg_buffer[3][buffer_index] = y1;
  avg_buffer[4][buffer_index] = z1;
  y2 = 0;
  y1 = 0;
  x = 0;
  z1 = 0;
  z2 = 0;

for(int i = 0; i<avg_buffer_size; i++){
  y2 += avg_buffer[0][i];
  z2 += avg_buffer[1][i];  
  x += avg_buffer[2][i];  
  y1 += avg_buffer[3][i];  
  z1 += avg_buffer[4][i];    
}

y2_final = y2/5;
y1_final = -(y1/5);
z2_final = z2/5;
z1_final = z1/5;
x_final = x/5;
delay(100);
//Serial.print(" Y1: ");
//Serial.print(y1_final);
//Serial.print("  Y2: ");
//Serial.print(y2_final);
//Serial.print(" Y1-Y2: ");
//Serial.println(y1_final - y2_final);
//if ((y1_final <0) && (y2_final < 0)){
//  Serial.print("counter");
//}
//else if((y1_final > 0) && (y2_final > 0)){
//  Serial.print("clock");
//}
//else{
//  Serial.print("nanda");
//}

//Serial.print(" Y1: ");
//Serial.print(y1_final-97000);
//Serial.print("  Y2: ");
//Serial.print(y2_final+72000);
//Serial.print(" y1-y2: ");
//Serial.println(-y1_final - y2_final + 25000);


}


void requestEvent(){
  //Serial.println("rec");
  get_readings();
  Wire.write(sentReadings, SENT_SIZE);
  bool state = digitalRead(13);
  digitalWrite(13, !state);
}

void get_readings(){
  sentReadings[3] = y2_final & 0xFF;
  sentReadings[2] = ((y2_final & (0xFF00))>>8);
  sentReadings[1] = ((y2_final & (0xFF0000))>>16);
  sentReadings[0] = ((y2_final & (0XFF000000))>>24);
  sentReadings[7] = (z2_final & (255));
  sentReadings[6] = (z2_final & (0xFF00))>>8;
  sentReadings[5] = (z2_final & (0xFF0000))>>16;
  sentReadings[4] = (z2_final & (0xFF000000))>>24;
  sentReadings[11] = (x_final & (255));
  sentReadings[10] = (x_final & (0xFF00))>>8;
  sentReadings[9] = (x_final & (0xFF0000))>>16;
  sentReadings[8] = (x_final & (0xFF000000))>>24;
  sentReadings[15] = (y1_final & (255));
  sentReadings[14] = (y1_final & (0xFF00))>>8;
  sentReadings[13] = (y1_final & (0xFF0000))>>16;
  sentReadings[12] = (y1_final & (0xFF000000))>>24;
  sentReadings[19] = (z1_final & (255));
  sentReadings[18] = (z1_final & (0xFF00))>>8;
  sentReadings[17] = (z1_final & (0xFF0000))>>16;
  sentReadings[16] = (z1_final & (0xFF000000))>>24;

}


