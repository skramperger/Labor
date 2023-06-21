#include <Arduino.h>
const byte LedPins[] = {2, 3, 4, 5, 6, 7, 11, 12, 9, 8};

#define HallsensorPin A0
#define CoilOutputPin 9

#define SinusEnable
//#define Testing_Serial

int hallMeasurement = 0;      // Sampled Hall Sensor value
int electromagnetSetting = 0; // PWM setting for electromagnet

// PID Values
float setpoint = 170; //200 AAA battery, 140 M6x60, 240 USB Stick
// 160 -- 216 effective up and down range   // 170 for screw

float setpointlow = 160;
float setpointhigh = 216;
float setpointdifference = 0;

unsigned long SinusSignalTime = 0;
int SinusSignalStep = 1;
unsigned int AccelerationFactor = 1;    // this value changes the speed at which sinus runs
float SinusCalculatedValue = 0;
float SinusValue = 0;
int PositiveNegativeSwitch = 1;
float calculatedmiddlevalue = 0;
float setpointlocation = 0;
float setpointmiddlevalue = 0;
float distance = 0, relation = 0;

float measured_value = 0;
float output = 0;
float integral = 0;
float derivative = 0;
float error = 0;
float previous_error = 0;
float dt = .1;
float Kp = 100.0;
float Ki = 500.0;
float Kd = 130;
float integralSaturation = 2000;
int pidLoopDelay = 10; // microseconds

// Additional Up and Down movement - variables
unsigned long previousMillis = 0;
const long interval = 500;   // interval at which to move up and down
const long interval_clearline = 1000; // interval for serial.monitor
unsigned long previousMillis_2 = 0;

void dumpPIDVariables();

void setup()
{
  Serial.begin(115200);
  Serial.println("Electromagnetic levitation demonstration.");
  Serial.println("Starting...");

  pinMode(CoilOutputPin, OUTPUT);

  for (byte i = 0; i < sizeof(LedPins); i++) { //LedPins is a byte so no need to devide the size
    pinMode(LedPins[i], OUTPUT);
  }

  for (byte i = 0; i < sizeof(LedPins); i++) { //LedPins is a byte so no need to devide the size
    digitalWrite(LedPins[i], HIGH);
    delay(40);
  }
  for (byte i = 0; i < sizeof(LedPins); i++) { //LedPins is a byte so no need to devide the size
    digitalWrite(LedPins[i], LOW);
    delay(40);
  }

  Serial.println("Started.");
}

void loop() // PID
{
    #ifdef SinusEnable

  setpointdifference = setpointhigh - setpointlow;
  setpointmiddlevalue = setpointlow + calculatedmiddlevalue;

  unsigned long ActualTime = millis();
  if(ActualTime > (SinusSignalTime + 20))
  {
    SinusSignalTime = ActualTime;
    calculatedmiddlevalue = (setpointdifference / 2); // Distance between setpointlow or setpointhigh to middle point

    if (setpoint > setpointmiddlevalue){
      distance = setpointhigh - setpoint;
    }
    else if (setpoint <= setpointmiddlevalue){
      distance = setpoint - setpointlow;
    }

    relation = distance / calculatedmiddlevalue;
    SinusValue = sin(relation);

    SinusCalculatedValue = SinusValue * AccelerationFactor;

    setpoint += SinusCalculatedValue * PositiveNegativeSwitch;

    if(setpoint+0.3 >= setpointhigh || setpoint-0.3 <= setpointlow)
    {
      PositiveNegativeSwitch = PositiveNegativeSwitch * -1;
    }
    #ifdef Testing_Sinus
    Serial.print(int(setpoint));
    Serial.print(" | ");
    Serial.println(SinusCalculatedValue);
    #endif
  }
  #endif

  
  while (Serial.available() > 0)
  {
    setpoint = Serial.parseInt();
    //Kp = Serial.parseInt();
    //Ki = Serial.parseInt();
    //Kd = Serial.parseInt();
    //dumpPIDVariables();

    while (Serial.available() > 0)
    {
      Serial.read();
    }
    Serial.println(setpoint);
    Serial.println(SinusCalculatedValue);
  }

  // Read Hall sensor
  measured_value = analogRead(HallsensorPin);


  // update the PID variables
  error = (setpoint - measured_value);
  integral = constrain(integral + error * dt, (integralSaturation * -1), integralSaturation);
  derivative = (error - previous_error) / dt;
  // Calculate the PWM value
  output = 0 - Kp * error - Ki * integral - Kd * derivative;
  // save the current error for the next iteration
  previous_error = error;
  // truncate and limit
  electromagnetSetting =  constrain(output, 0, 255); ;

  if (measured_value > 320)
  {
    electromagnetSetting = 0;
  }

  // apply the electromagnet setting
  analogWrite(CoilOutputPin, electromagnetSetting);

  // Time between electromagnet state changes;
  delayMicroseconds(pidLoopDelay);

}
