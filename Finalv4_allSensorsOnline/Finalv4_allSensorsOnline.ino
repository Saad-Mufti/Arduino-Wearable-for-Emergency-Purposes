//This version includes a very basic implementation of the pulse sensor, and a more advanced reading should be expected in the next version...

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

const int AOUTpin = 0; //GAS ANALOG PIN
const int DOUTpin = 1; //GAS DIGITAL PIN
const int ledPin = 13; //LED FOR GAS (DIGITAL 13)
int gasLimit; //GAS
int value; //GAS
//int sensorPin = A5; // TEMP
int val;
int tempPin = 4;
boolean LM35_1 = false;

const int PulseWire = 0;
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
  // Try to enable GPRS


  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);
  pinMode(DOUTpin, INPUT);//TURNING GAS DIGITAL PIN TO OUTPUT
  pinMode(ledPin, OUTPUT);//TURNING LED DIGITAL PIN TO OUTPUT
  //  sendSMS();
  Serial.println("Boot-up successful");
  if (pulseSensor.begin()) {
    Serial.println("Pulse Online");
  }
  Serial.println("Please put on and secure the device on your device. In exactly 5 seconds, the heart sensor will calibrate your pulse to determine your heart rate at a normal level.");
  delay(5000);
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);

  //  normalValueInitialization();
}

void loop() {
  int myBPM = pulseSensor.getBeatsPerMinute();
  for (int a = 0; a < 40; a++) {
    normalHeartRate[a] = 1; //"1" needs to replaced with the variable that receives the heart rate input
  }
  if (pulseSensor.sawStartOfBeat()) {
    Serial.print("BPM: ");
    Serial.println(myBPM);
  }
  else Serial.println("No pulse detected");

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


  if (farh > 150 || farh < -10) {
    (LM35_1 = true);
    Serial.println("Warning 1 for LM35");
  }
  else {
    Serial.println("LM35 indicates safety");
  }
  //END OF TEMP BODY
  if (value > 500) {
    gasWarning = true;
    Serial.println("Warning for CO sensor");
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
  char sendto[21] = {'5', '0', '8', '3', '6', '1', '8', '8', '1', '1'};
  char message[141] = {'t', 'e', 's', 't'}; //141 characters max
  //Remember, "" denotes a String datatype, whereas a '' denotes a char datatype
  Serial.print(F("Send to #"));
  //  readline(sendto, 20); //NEED TO FIGURE OUT //FIGURED OUT!
  Serial.println(sendto);
  Serial.print(F("Type out one-line message (140 char): "));
  //  readline(message, 140); //NEED TO FIGURE OUT
  Serial.println(message);
  if (fona.sendSMS(sendto, message)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Message sent"));
  }
}
