/*
using sparkfun esp32 lot Redboard

*/


const int analog_pin_in1 = 32;
const int analog_pin_in2 = 33;

void setup() {
  Serial.begin(115200);
  pinMode(analog_pin_in1, INPUT);
  pinMode(analog_pin_in2, INPUT);
  Serial.println("Readings:");
}

void loop() {
  Serial.println(analogRead(analog_pin_in2) - analogRead(analog_pin_in1));
  delay(200);
}
