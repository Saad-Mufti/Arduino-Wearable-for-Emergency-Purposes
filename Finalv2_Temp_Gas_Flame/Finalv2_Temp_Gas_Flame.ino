// #include <math.h> //ADVANCED MATH FUNCTIONS //not needed anymore
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h> //PULSE

const int AOUTpin = 0; //GAS ANALOG PIN
const int DOUTpin = 1; //GAS DIGITAL PIN
const int ledPin = 13; //LED FOR GAS (DIGITAL 13)
int gasLimit; //GAS
int value; //GAS
//int sensorPin = A5; // TEMP
int val;
int tempPin = 5;
boolean LM35_1 = false;

const int OUTPUT_TYPE = SERIAL_PLOTTER; //PULSE
const int sensorMin = 0; //FLAME
const int sensorMax = 1024;  //FLAME
boolean flameWarningHigh = false; //FLAME
boolean flameWarningLow = false; //FLAME

double normalHeartRate[40];

void setup() {
  Serial.begin(9600);
  Serial.println("Boot-up successful");
  Serial.println("Please put on and secure the device on your device. In exactly 5 seconds, the heart sensor will calibrate your pulse to determine your heart rate at a normal level.");
  delay(5000);
  for (int a = 0; a < 40; a++) {
    normalHeartRate[a] = 1; //"1" needs to replaced with the variable that receives the heart rate input
  }
  pinMode(DOUTpin, INPUT);//TURNING GAS DIGITAL PIN TO OUTPUT
  pinMode(ledPin, OUTPUT);//TURNING LED DIGITAL PIN TO OUTPUT
}

void loop() {

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

  //  void normalValueInitialization();

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


  Serial.println("-------------------------------------");

}


boolean gasWarning = false;
double tempData[20];
double gasData[20];
int gasValueHolder = 0;

void normalValueInitialization() {
  for (int i = 0; i < 20; i++) {
    //    tempData[i] = temp;
    gasData[i] = value;
    gasValueHolder += gasData[i];
    Serial.println(gasValueHolder);
    delay(1000);
  }
}








