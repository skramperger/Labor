#include <Arduino.h>

#define button 13
int LED[] = {11, 7, 6};

// Berechnung von Größe des Arrays
size_t n = sizeof(LED) / sizeof(LED[0]);


void setup() {
  // put your setup code here, to run once:

  pinMode(button, INPUT);

  for (int i=0; i<n; i++)
  {
    pinMode(LED[i], OUTPUT);
  }

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i=0; i<n; i++)
  {
    digitalWrite(LED[i], digitalRead(button));
    delay(1000);
    digitalWrite(LED[i], LOW);
  }
}
