#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h> //PULSE

#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
String latString;
String longString;
String coordinates;

const int AOUTpin = 0; //GAS ANALOG PIN
const int DOUTpin = 1; //GAS DIGITAL PIN
const int ledPin = 13; //LED FOR GAS (DIGITAL 13)
int gasLimit; //GAS
int value; //GAS
int val;
int tempPin = 4;
boolean LM35_1 = false;

const int PulseWire = 1;
int Threshold = 550;
PulseSensorPlayground pulseSensor;

const int sensorMin = 0; //FLAME
const int sensorMax = 1024;  //FLAME
boolean flameWarningHigh = false; //FLAME
boolean flameWarningLow = false; //FLAME

double normalHeartRate[40];


void setup() {
  Serial.begin(115200);
  while (! Serial);
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));
  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);

  pinMode(DOUTpin, INPUT);//TURNING GAS DIGITAL PIN TO OUTPUT
  pinMode(ledPin, OUTPUT);//TURNING LED DIGITAL PIN TO OUTPUT


  Serial.println("Boot-up successful");
  /*     //GPS doesn't like this block
    if (pulseSensor.begin()) {
    Serial.println("Pulse Online");
    } */
  Serial.println("Please put on and secure the device on your device. In exactly 5 seconds, the heart sensor will calibrate your pulse to determine your heart rate at a normal level.");
  delay(5000);
  //  normalValueInitialization();
}

void loop() {
  float latitude, longitude, speed_kph, heading, speed_mph, altitude;
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  latString = String(latitude, 6);
  longString = String(longitude, 6);
  coordinates = latString + "\n" + longString;
  if (gps_success) {
    Serial.print("GPS lat:");
    Serial.println(latitude, 6);
    Serial.print("GPS long:");
    Serial.println(longitude, 6);
    Serial.print("GPS speed KPH:");
    Serial.println(speed_kph);
    Serial.print("GPS speed MPH:");
    speed_mph = speed_kph * 0.621371192;
    Serial.println(speed_mph);
    Serial.print("GPS heading:");
    Serial.println(heading);
    Serial.print("GPS altitude:");
    Serial.println(altitude);
  } else {
    Serial.println("GPS Fix Failed");
  }
  int pulseRead = analogRead(PulseWire);
  if (pulseRead > Threshold) {
    Serial.print("Pulse:"); Serial.println(pulseRead);
  }
  else {
    Serial.println("No pulse found.");
  }
  for (int a = 0; a < 40; a++) {
    normalHeartRate[a] = 1; //"1" needs to replaced with the variable that receives the heart rate input
  }


  int sensorReading = analogRead(A1); //FLAME
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3); //FLAME

  switch (range) {
    case 0:    // A fire closer than 1.5 feet away.
      Serial.println(" "); Serial.println("** Close Fire **");
      Serial.println(sensorReading); Serial.println(" "); Serial.println(" ");
      flameWarningHigh = true;
      break;
    case 1:    // A fire between 1-3 feet away.
      Serial.println(" "); Serial.println("** Distant Fire **");
      Serial.println(sensorReading); Serial.println(" "); Serial.println(" ");
      flameWarningLow = true;
      break;
    case 2:    // No fire detected.
      Serial.println(" "); Serial.println("No Fire");
      Serial.println(sensorReading); Serial.println(" "); Serial.println(" ");
      break;
  }
  delay(2000);

  boolean gasWarning = false;
  value = analogRead(AOUTpin); //GAS ANALOG
  gasLimit = digitalRead(DOUTpin); //GAS DIGITAL
  Serial.print("CO value: "); Serial.print(value); //CO = CARBON MONOXIDE
  Serial.println(" ");
  Serial.print("CO gasLimit: "); Serial.print(gasLimit);
  Serial.println("");
  delay(100);
  if (gasLimit == HIGH) {
    digitalWrite(ledPin, HIGH);//if limit has been reached, LED turns on as status indicator
  }
  else {
    digitalWrite(ledPin, LOW);//if threshold not reached, LED remains off
  }

  //TEMP BODY
  val = analogRead(tempPin);
  float mv = ( val / 1024.0) * 5000;
  float cel = mv / 10;
  float farh = (cel * 9) / 5 + 32;

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


  if (farh > 120 || farh < -5) {
    (LM35_1 = true);
    Serial.println("Warning 1 for LM35");
  }
  else {
    Serial.println("LM35 indicates safety");
  }
  //END OF TEMP BODY
  if (value > 550) {
    gasWarning = true;
    Serial.println("Warning for CO sensor");
  }

if (gasWarning && tempWarningMAX && flameWarningHigh) {
  sendSMS();
  Serial.println("Emergency detected \n Calling emergency personell");
}

  Serial.println("-------------------------------------");

}


boolean gasWarning = false;
double tempData[20];
double gasData[20];
int tempValueHolder = 0;
int gasValueHolder = 0;

void normalValueInitialization() {
  for (int i = 0; i < 20; i++) {
    //   tempData[i] = farh;
    gasData[i] = value;
    gasValueHolder += gasData[i];
    tempValueHolder += tempData[i];
    Serial.println(gasValueHolder);
    Serial.println(tempValueHolder);
    delay(1000);
    if (i = 20) {
      int gasAver = gasValueHolder / i;
      int tempAver = tempValueHolder / i;
      Serial.println("Sensor values initialized");
    }
  }
}
void sendSMS() {
  String myPhoneNumber = "5083618811";
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

