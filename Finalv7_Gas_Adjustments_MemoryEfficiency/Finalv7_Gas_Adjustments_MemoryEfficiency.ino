/*
  Flame Pins: A2, D8
  Gas Pins: A0, D5
  Temp Pins: A4
  GSM: D2 (TX), D3(RX), D4 (RST)
  LEDs: D12 Green (O+,Y-), D11 Red
  Button: D13
  Pulse: A1
*/
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

const byte FONA_RX = 3;
const byte FONA_TX = 2;
const byte FONA_RST = 4;
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
String latString;
String longString;
String coordinates;

const byte AOUTpin = 0; //GAS ANALOG PIN
const byte DOUTpin = 5; //GAS DIGITAL PIN
byte gasLimit; //GAS
int value; //GAS ANALOG

short val; //TEMP ANALOG READ
const byte tempPin = 4; //TEMP ANALOG
boolean LM35_1 = false;
int i = 0;
float tempAvg = 0;
float farh;
boolean tempDetached;

const int PulseWire = 1;
int Threshold = 550;
PulseSensorPlayground pulseSensor;
double normalHeartRate[40];

byte flameDigital; //FLAME DIGITAL READ
const byte flameAnalog = 2; //FLAME ANALOG PIN
const byte DOFlame = 8; //FLAME DIGITAL PIN
short Output; //FLAME ANALOG
boolean flameWarningHigh = false; //FLAME
boolean flameWarningLow = false; //FLAME

const byte GLED = 12;
const byte RLED = 11;

void setup() {
  Serial.begin(115200);
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  while (! Serial);
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    digitalWrite(RLED, HIGH);
    while (1);
  }
  Serial.println(F("FONA is OK"));
  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);

  pinMode(DOUTpin, INPUT);//TURNING GAS DIGITAL PIN TO INPUT
  pinMode(DOFlame, INPUT);//FLAME DIGITAL TO INPUT

  Serial.println(F("Boot-up successful"));
  Serial.println(F("Please put on and secure the device on your device. In exactly 5 seconds, the heart sensor will calibrate your pulse to determine your heart rate at a normal level."));
  delay(5000);
}

void loop() {

  //GPS BODY
  delay(1000);
  float latitude, longitude, speed_kph, heading, speed_mph, altitude;
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  latString = String(latitude, 6);
  longString = String(longitude, 6);
  coordinates = latString + "\n" + longString;
  if (gps_success) {
    digitalWrite(GLED, HIGH);
    Serial.print(F("GPS lat:"));
    Serial.println(latitude, 6);
    Serial.print(F("GPS long:"));
    Serial.println(longitude, 6);
  } else {
    Serial.println("GPS Fix Failed");
//    redBlinkSlow();
    if (latitude != 0.0) {
      Serial.println(F("Last known location:"));
      Serial.print(F("GPS lat:"));
      Serial.println(latitude, 6);
      Serial.print(F("GPS long:"));
      Serial.println(longitude, 6);
    }
  }
  //END OF GPS BODY

  //PULSE BODY
  int pulseRead = analogRead(PulseWire);
  if (pulseRead > Threshold) {
    Serial.print(F("Pulse:"));
    Serial.print(pulseRead);
  }
  else {
    Serial.println(F("No pulse found."));
  }
  for (int a = 0; a < 40; a++) {
    normalHeartRate[a] = 1; //"1" needs to replaced with the variable that receives the heart rate input
  }
  //END OF PULSE BODY

  //FLAME BODY
  Output = analogRead(flameAnalog);
  flameDigital = digitalRead(DOFlame);
  if (flameDigital == HIGH)  {
    Serial.println("FIRE");
  }
  else {
    Serial.print(F("No Fire Detected  "));
  }
  Serial.println(Output);
  //END OF FLAME BODY


  //GAS BODY
  boolean gasWarning = false;
  value = analogRead(AOUTpin); //GAS ANALOG
  gasLimit = digitalRead(DOUTpin); //GAS DIGITAL
  Serial.print(F("CO value: "));
  Serial.print(value); //CO = CARBON MONOXIDE
  Serial.println(" ");
  Serial.print(F("CO gasLimit: "));
  Serial.println(gasLimit);

  if (gasLimit == HIGH) {
    gasWarning = true;
    Serial.println(F("Warning from gas sensor"));
  }
  //END OF GAS BODY

  //TEMP BODY
  val = analogRead(tempPin);
  float mv = ( val / 1024.0) * 5000;
  float cel = mv / 10;
  farh = (cel * 9) / 5 + 32;

  boolean tempWarningHigh = false;
  boolean tempWarningMAX = false;
  if (farh < 0 || farh > 120) {
    tempWarningHigh = true;
  }
  if (farh < -10 || farh > 150) {
    tempWarningMAX = true;
  }

  Serial.print("TEMPRATURE = ");
  Serial.print(farh);
  Serial.print("*F");
  Serial.println();
  float tempData[200];
  tempData[i] = farh;
  tempAvg += tempData[i];
  float betterFarh = tempAvg / (i + 1);
  Serial.print(F("Temperature average so far:  "));
  Serial.println(betterFarh);
  i++;

  if (betterFarh > 120 || betterFarh < -5) {
    (LM35_1 = true);
    Serial.println(F("Warning 1 for LM35"));
  }
  else {
    Serial.println(F("LM35 indicates safety"));
  }
  if (betterFarh > 200) {
    Serial.println(F("Temperature sensor dysfunctional. Please adjust accordingly"));
    tempDetached = true;
//    redBlinkSlow();
  }
  //END OF TEMP BODY

  if (!tempDetached) {
    if (gasWarning && tempWarningMAX) {
//      EmergencyLight();
      sendSMS();
      Serial.println(F("Emergency detected \n Calling emergency personell"));
    }
  }
  
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, LOW);
    Serial.println(buttonState);
    z = 0;
  } else {
    // turn LED off:
    digitalWrite(ledPin, HIGH);
    z = + z + 1;

  Serial.println(F("-----------------------------------------------"));
}


double tempData[20];
double gasData[20];
int tempValueHolder = 0;
int gasValueHolder = 0;

void normalValueInitialization() {
  for (int k = 0; k < 20; k++) {
    tempData[k] = farh;
    gasData[k] = value;
    gasValueHolder += gasData[k];
    tempValueHolder += tempData[k];
    Serial.println(gasValueHolder);
    Serial.println(tempValueHolder);
    delay(1000);
    if (k = 20) {
      int gasAver = gasValueHolder / k;
      int tempAver = tempValueHolder / k;
      Serial.println(F("Sensor values initialized"));
    }
  }
}

void sendSMS() {
  String myPhoneNumber = F("5083618811"); //PHONE # GOES HERE
  char sendto[21];
  myPhoneNumber.toCharArray(sendto, 21);
  char message[141];
  coordinates.toCharArray(message, 141);
  //Remember, "" denotes a String datatype, whereas a '' denotes a char datatype
  Serial.print(F("Send to #"));
  Serial.println(sendto);
  Serial.println(message);
  if (fona.sendSMS(sendto, message)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent"));
  }
}

void redBlinkSlow() {
  for (int g = 0; g < 4; g++) {
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, HIGH);
    delay(825);
    digitalWrite(RLED, LOW);
    delay(825);
  }
}
void EmergencyLight() {
  //  pinMode(GLED, INPUT);
  digitalWrite(GLED, LOW);
  for (int i = 0; i < 100; i++) {
    digitalWrite(RLED, HIGH);
    delay(100);
    digitalWrite(RLED, LOW);
    delay(100);
  }
}
