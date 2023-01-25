#include <Arduino.h>

#define redLED 2
#define pinButton 13

void setup() {
  // put your setup code here, to run once:
  pinMode(pinButton, INPUT);
  pinMode(redLED, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(redLED, digitalRead(pinButton));
}
