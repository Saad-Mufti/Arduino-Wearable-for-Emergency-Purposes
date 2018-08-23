

int val;
int tempPin = 5;
boolean LM35_1 = false;

const int AOUTpin = 0;
const int DOUTpin = 8;
int limit;
int value;


void setup() {
  Serial.begin(9600);
  Serial.println("bootup successful");
  Serial.println("In exactly 5 seconds, the pulse sensor will calibrate and read your pulse to know your normal heartrate");
  pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
  delay(5000);
}

void loop() {
  val = analogRead(tempPin);
  float mv = ( val / 1024.0) * 5000;
  float cel = mv / 10;
  float farh = (cel * 9) / 5 + 32;
  Serial.print("TEMPRATURE = ");
  Serial.print(farh);
  Serial.print("*F");
  Serial.println();
  if (farh > 150 || farh < -10) {
    (LM35_1 = true);
    Serial.println("Warning 1 for LM35");
    Serial.println("");
  }
  else {
    Serial.println("LM35 indicates safety");
    Serial.println("");
    return;
  }

  value = analogRead(AOUTpin);//reads the analaog value from the CO sensor's AOUT pin
  limit = digitalRead(DOUTpin);//reads the digital value from the CO sensor's DOUT pin
  Serial.print("CO value: ");
  Serial.println(value);//prints the CO value
  Serial.print("CO Limit: ");
  Serial.println(limit);//prints the limit reached as either LOW or HIGH (above or underneath)

  Serial.println("----------------------------------------------");
  delay(1000);










}
