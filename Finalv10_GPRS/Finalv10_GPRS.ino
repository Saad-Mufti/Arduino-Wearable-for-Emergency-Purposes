/*
  Flame Pins: A2, D8
  Gas Pins: A0, D5
  Temp Pins: A4
  GSM: D2 (TX), D3(RX), D4 (RST)
  LEDs: D12 Green, D11 Red
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
String EmergencyMessage;
String manualEmergency;
String warningGas;
String warningTempHigh;
String WarningTempLow;
bool Attached;

const byte AOUTpin = 0; //GAS ANALOG PIN
const byte DOUTpin = 5; //GAS DIGITAL PIN
byte gasLimit; //GAS
int value; //GAS ANALOG
boolean gasWarning = false;

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
//short Output; //FLAME ANALOG
boolean flameWarningHigh = false; //FLAME
boolean flameWarningLow = false; //FLAME
boolean flameWarning = false;

boolean tempWarningHigh;
boolean tempWarningLow = false;

const byte GLED = 12;
const byte RLED = 11;

const byte RBUTTON = 7;
const byte BBUTTON = 13;
byte z = 0;
int BlackRead;
int RedRead;
bool manualStop;

int Time;
int lastGPSread;

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
  fona.setGPRSNetworkSettings(F("fast.t-mobile.com"));
  fona.enableGPS(true);

  pinMode(DOUTpin, INPUT);//TURNING GAS DIGITAL PIN TO INPUT
  pinMode(DOFlame, INPUT);//FLAME DIGITAL TO INPUT

  pinMode(BBUTTON, INPUT);
  pinMode(RBUTTON, INPUT);

  Serial.println(F("Boot-up successful"));
  manualEmergency = F("Emergency: User is requesting medical attention \n Please send help immediately \n");
  warningGas = F("Warning: Lethal Carbon monoxide level increase detected. \n Location: \n");
  warningTempHigh = F("Warning: Extreme high temperature detected. \n Location: \n");
  WarningTempLow = F("Warning: Extreme low temperaure detected. \n Location: \n");
  Attached = false;
  delay(1000);
}

void loop() {
  manualStop = false;
  //GPS BODY
  delay(750);
  //EmergencyMessage = coordinates;
  float latitude, longitude;
  boolean gps_success = fona.getGPS(&latitude, &longitude);
  latString = String(latitude, 6);
  longString = String(longitude, 6);
  coordinates = latString + "\n" + longString;
  //  time = millis();
  // if (i >= 1) {
  //  if((time - lastGPSread) < 30000) {
  //
  if (gps_success) {
    digitalWrite(GLED, HIGH);
    Serial.print(F("GPS lat:"));
    Serial.println(latitude, 6);
    Serial.print(F("GPS long:"));
    Serial.println(longitude, 6);
    lastGPSread = millis();
  }
  else {
    Serial.println("GPS Fix Failed");
    redBlinkSlow();
    if (latitude != 0.0) {
      Serial.println(F("Last known location:"));
      Serial.print(F("GPS lat:"));
      Serial.println(latitude, 6);
      Serial.print(F("GPS long:"));
      Serial.println(longitude, 6);
    }
  }
  uint8_t n = fona.getRSSI();
  int8_t r;

  Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  Serial.print(r); Serial.println(F(" dBm"));

  //}
  //END OF GPS BODY

  pulseBody();

  flameBody();

  gasBody();

  tempBody();

  emergencyCheck();

  BlackRead = digitalRead(BBUTTON);
  if (BlackRead == HIGH) {
    Serial.println("Button pressed");
    Serial.println(z);
    z = z + 1;
    if (z >= 3) {
      Serial.println(F("EMERGENCY"));
      //Serial.println(manualEmergency);
      //Serial.println(coordinates);
      EmergencyMessage = manualEmergency + "\n" + coordinates;
      //manualEmergency += coordinates;
      Serial.println(EmergencyMessage);
      sendSMS();
      EmergencyLight();
    }

  } else {
    z = 0;
  }
  manualEmergency = "";
  Serial.println(F("-----------------------------------------------"));
}

void sendSMS() {
  String myPhoneNumber = F("5083618811"); //PHONE # GOES HERE
  char sendto[21];
  myPhoneNumber.toCharArray(sendto, 21);
  char message[141]; //141
  manualEmergency.toCharArray(message, 141); //141
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
  for (int g = 0; g < 3; g++) {
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, HIGH);
    delay(680);
    digitalWrite(RLED, LOW);
    delay(680);
  }
}
void EmergencyLight() {
  //  pinMode(GLED, INPUT);
  digitalWrite(GLED, LOW);
  for (int i = 0; i < 100; i++) {
    digitalWrite(RLED, HIGH);
    delay(90);
    digitalWrite(RLED, LOW);
    delay(90);
  }
}
void ManualStop() {
  for (int p = 0; p < 3000; p++) {
    RedRead = digitalRead(RBUTTON);
    if (RedRead == HIGH) {
      manualStop = true;
    }
  }
}

void emergencyCheck() {
  if (!tempDetached) {
    if (tempWarningLow) {
      EmergencyMessage = WarningTempLow + coordinates;
      EmergencyLight();
      sendSMS();
      Serial.println(F("Cold Emergency detected \n Calling emergency personell"));
    }
    if (tempWarningHigh && flameWarning) {
      EmergencyMessage = warningTempHigh;
      sendSMS();
      Serial.println(F("High temperature/fire detected /n calling emergency personnel"));
      EmergencyLight();
    }
    if (gasWarning) {
      EmergencyMessage = warningGas + coordinates;
      Serial.println(F("Gas Emergency detected \n Calling emergency personell"));
      sendSMS();
    }
  }
}

void tempBody() {
  val = analogRead(tempPin);
  float mv = ( val / 1025.0) * 5000; //1024
  float cel = (mv / 10) / 3.514; // divide by 3.514
  farh = (cel * 9) / 5 + 32;

  tempWarningHigh = false;
  //  boolean tempWarningMAX = false;
  tempWarningLow = false;
  if (farh > 120) {
    tempWarningHigh = true;
  }
  if (farh < 0) {
    tempWarningLow = true;
  }
  //  if (farh < -10 || farh > 150) {
  //    tempWarningMAX = true;
  //  }

  Serial.print("TEMPRATURE = ");
  Serial.print(farh);
  Serial.println("*F");
  float tempData[300];
  tempData[i] = farh;
  tempAvg += tempData[i];
  float betterFarh = tempAvg / (i + 1);
  Serial.print(F("Temperature average so far:  "));
  Serial.println(betterFarh);
  i++;

  int tempDif = tempData[i] - tempData[i - 1];
  if (i > 3) {
    if (tempDif > 30) {
      Serial.println(F("Temperature sensor dysfunctional. Please adjust accordingly"));
      tempDetached = true;
    }
  }

  if (betterFarh > 120 || betterFarh < -5) {
    LM35_1 = true;
    Serial.println(F("Warning 1 for LM35"));
  }
  else {
    Serial.println(F("LM35 indicates safety"));
  }
}

void gasBody() {
  gasWarning = false;
  //  value = analogRead(AOUTpin); //GAS ANALOG
  gasLimit = digitalRead(DOUTpin); //GAS DIGITAL
  //  Serial.print(F("CO value: "));
  //  Serial.print(value); //CO = CARBON MONOXIDE
  //  Serial.println(" ");
  //  Serial.print(F("CO gasLimit: "));
  //  Serial.println(gasLimit);

  if (gasLimit == HIGH) {
    gasWarning = true;
    Serial.println(F("Warning from gas sensor"));
  }
  else {
    Serial.println(F("Safety for Gas Sensor"));
  }
}
void pulseBody() {
  int pulseRead = analogRead(PulseWire);
  if (pulseRead > Threshold) {
    Serial.print(F("Pulse:"));
    int pulse = pulseRead / 12;
    Serial.print(pulse);
    Attached = true;
    for (int a = 0; a < 40; a++) {
      normalHeartRate[a] = pulseRead;
    }    //doesnt show change
  }
  else {
    Serial.println(F("No pulse found."));
    Attached = false;
  }
}

void flameBody() {
  flameWarning = false;
  //  Output = analogRead(flameAnalog);
  flameDigital = digitalRead(DOFlame);
  if (flameDigital == HIGH)  {
    Serial.println("FIRE");
    flameWarning = true;
  }
  else {
    Serial.print(F("No Fire Detected"));
  }
  //  Serial.println(Output);
}

