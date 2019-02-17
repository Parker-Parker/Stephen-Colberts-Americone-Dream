void setup() {
Serial.begin(9600); // set the baud rate
 Serial.println("Ready"); // print "Ready" once
}
char s1;
void loop() {
  if (Serial.available())
  {
  s1 = Serial.read();
  Serial.println(s1);
  }
  
}
