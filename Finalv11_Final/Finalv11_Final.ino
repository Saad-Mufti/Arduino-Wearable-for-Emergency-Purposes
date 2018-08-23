/*
  Flame Pins: A2, D8
  Gas Pins: A0, D5+
  *
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
float latitude, longitude;
String latString;
String longString;
String coordinates;
char replybuffer[255];
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

short val; //TEMP ANALOG READ
const byte tempPin = 4; //TEMP ANALOG
boolean LM35_1 = false;
int i = 0;
float tempAvg = 0;
float farh;
boolean tempDetached;

byte PulseWire = 1;
int lowThreshold = 555;
int highThreshold = 850;
int pulseRead;
int pulse;
int pulseSum;
int q;
int pulseAverage;
bool pulseWithinthresh1;
bool pulseWithinthresh2;

byte flameDigital; //FLAME DIGITAL READ
const byte flameAnalog = 2; //FLAME ANALOG PIN
const byte DOFlame = 8; //FLAME DIGITAL PIN
short Output; //FLAME ANALOG
boolean flameWarningHigh = false; //FLAME
boolean flameWarningLow = false; //FLAME

const byte GLED = 12;
const byte RLED = 11;

const byte RBUTTON = 7;
const byte BBUTTON = 13;
byte z = 0;
int BlackRead;
int RedRead;
bool manualStop;

int time;
int lastGPSread;

void setup() {
  Serial.begin(115200);
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  digitalWrite(RLED, LOW);
  digitalWrite(GLED, LOW);
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
//  fona.setGPRSNetworkSettings(F("fast.t-mobile.com"));
  pinMode(DOUTpin, INPUT);//TURNING GAS DIGITAL PIN TO INPUT
  pinMode(DOFlame, INPUT);//FLAME DIGITAL TO INPUT

  pinMode(BBUTTON, INPUT);
  pinMode(RBUTTON, INPUT);

  Serial.println(F("Boot-up successful"));
  manualEmergency = F("Manual Emergency \n");
  warningGas = F("CO Increase \n Location: \n");
  warningTempHigh = F("Extreme high temperature detected. \n Location: \n");
  WarningTempLow = F("Extreme low temperaure detected. \n Location: \n");
  Attached = false;

  do {
    if (!fona.enableGPRS(true))
      Serial.println(F("Failed to turn on"));
  }
  while (!fona.enableGPRS(true));
  delay(1000);
}

void loop() {
  manualStop = false;
  //GPS BODY
  delay(750);
  //EmergencyMessage = coordinates;
  boolean gps_success = fona.getGPS(&latitude, &longitude);
  latString = String(latitude, 6);
  longString = String(longitude, 6);
  coordinates = latString + "\n" + longString;

  if (gps_success) {
    Serial.println("\n");
    digitalWrite(GLED, HIGH);
    Serial.print(F("GPS lat:"));
    Serial.println(latitude, 6);
    Serial.print(F("GPS long:"));
    Serial.println(longitude, 6);
    lastGPSread = millis();
  }
  else {
    Serial.println("GPS Fix Failed");
    if (latitude != 0.0) {
      if (fona.getNetworkStatus() == 1) {
        boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);
        Serial.print(F("GSM lat:"));
        Serial.println(latitude, 6);
        Serial.print(F("GSM long:"));
        Serial.println(longitude, 6);
      }
      Serial.println(F("Last known location:"));
      Serial.print(F("GPS lat:"));
      Serial.println(latitude, 6);
      Serial.print(F("GPS long:"));
      Serial.println(longitude, 6);

    }
    else {
      if (fona.getNetworkStatus() == 1) {
        boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);
        Serial.print(F("GSM lat:"));
        Serial.println(latitude, 6);
        Serial.print(F("GSM long:"));
        Serial.println(longitude, 6);
      }
      else {
        uint16_t returncode;
        if (!fona.getGSMLoc(&returncode, replybuffer, 250))
          Serial.println(F("Failed!"));
        if (returncode == 0) {
          Serial.println(replybuffer);
          for (int o; o < 10; o++) {
            longString += replybuffer[o];
          }
          for (int f; f < 9; f++) {
            latString += replybuffer[f + 14];
          }
          Serial.print(F("Backup lon")); Serial.print(longString); Serial.print(latitude);
          Serial.print(F("Backup lat")); Serial.print(latString); Serial.print(longitude);
        } else {
          redBlinkSlow();
          Serial.print(F("Fail code #")); Serial.println(returncode);
          Serial.println(replybuffer);
          for (int o; o < 10; o++) {
            longString += replybuffer[o];
          }
          for (int f; f < 9; f++) {
            latString += replybuffer[f + 14];
          }
          Serial.print("Lon: "); Serial.println(longString);
          Serial.print("Lat: "); Serial.println(latString);
        }
      }
    }
  }
  //}
  //END OF GPS BODY

  //PULSE BODY
  pulseWithinthresh1 = lowThreshold < pulseRead;
  pulseWithinthresh2 = pulseRead < highThreshold;
  pulseRead = analogRead(PulseWire);
  if (pulseWithinthresh1 && pulseWithinthresh2) {
    Serial.println(pulseRead);
    pulse = pulseRead / 8;
    q++;
    Serial.print("pulse:"); Serial.println(pulse);
    pulseSum += pulse;
    pulseAverage = pulseSum / i;
    Serial.print("Average: "); Serial.println(pulseAverage);
  }
  else {
    Serial.print("no pulse found");
    Serial.println(pulseRead);
    Attached = true;
  }

  //END OF PULSE BODY

  //FLAME BODY
  bool flameWarning = false;
  Output = analogRead(flameAnalog);
  flameDigital = digitalRead(DOFlame);
  if (flameDigital == HIGH)  {
    Serial.println("FIRE");
    flameWarning = true;
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
  float mv = ( val / 1025.0) * 5000; //1024
  float cel = (mv / 10) / 3.514; // divide by 3.514
  farh = (cel * 9) / 5 + 32;

  boolean tempWarningHigh = false;
  //  boolean tempWarningMAX = false;
  boolean tempWarningLow = false;
  if (farh > 130) {
    tempWarningHigh = true;
  }
  if (farh < 0) {
    tempWarningLow = true;
  }

  Serial.print(F("TEMPRATURE = "));
  Serial.print(farh);
  Serial.println(F("*F"));
  float tempData[300];
  tempData[i] = farh;
  tempAvg += tempData[i];
  float betterFarh = tempAvg / (i + 1);
  Serial.print(F("Temperature average so far:  "));
  Serial.println(betterFarh);
  i++;

  //  if (i > 3) {
  //    if (tempData[i] - tempData[i - 1] > 30) {
  //      Serial.println(F("Temperature sensor dysfunctional. Please adjust accordingly"));
  //      tempDetached = true;
  //    }
  //  }

  if (betterFarh > 120 || betterFarh < -5) {
    LM35_1 = true;
    Serial.println(F("Warning 1 for LM35"));
  }
  else {
    Serial.println(F("LM35 indicates safety"));
  }
  //END OF TEMP BODY

  if (!tempDetached) {
    if (tempWarningLow) {
      EmergencyMessage = WarningTempLow + coordinates;
      EmergencyLight();
      sendSMS();
      Serial.println(F("Cold Emergency detected \n Calling emergency personell"));
    }
    if (tempWarningHigh && flameWarning) {
      EmergencyMessage = warningTempHigh;
      Serial.println(F("High temperature/fire detected \n calling emergency personnel"));
      sendSMS();
      EmergencyLight();
    }
    if (gasWarning) {
      EmergencyMessage = warningGas + coordinates;
      Serial.println(F("Gas Emergency detected \n Calling emergency personell"));
      sendSMS();
    }
  }
  BlackRead = digitalRead(BBUTTON);
  if (BlackRead == HIGH) {
    Serial.println("Button pressed");
    Serial.println(z);
    z = z + 1;
    if (z >= 3) {
      ManualStop();
      if (manualStop == true) {
        return;
      }
      Serial.println(F("EMERGENCY"));
//      Serial.println(manualEmergency);
//      Serial.println(coordinates);
      EmergencyMessage = manualEmergency + coordinates;
      //manualEmergency += coordinates;
      Serial.println(EmergencyMessage);
      sendSMS();
      EmergencyLight();
    }

  } else {
    z = 0;
    Serial.println("Button not pressed :(");
  }
  Serial.println(F("-----------------------------------------------"));
}

void sendSMS() {
  String myPhoneNumber = F("5083618811"); //PHONE # GOES HERE
  char sendto[21];
  myPhoneNumber.toCharArray(sendto, 21);
  char message[141]; //141
  //manualEmergency.toCharArray(message, 141); //141
  EmergencyMessage.toCharArray(message, 141);
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
  //  noInterrupts();
  for (int g = 0; g < 3; g++) {
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, HIGH);
    delay(500);
    digitalWrite(RLED, LOW);
    delay(500);
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
      return;
    }
  }
}
