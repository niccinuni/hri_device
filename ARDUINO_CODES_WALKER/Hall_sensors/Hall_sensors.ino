const int hallPin2 = 32; //dx
const int hallPin1 = 34;//sx
const int hallPin3 = 35;//vtx

const int deadmanSwitchPin = 13;
const int motorForwardPin  = 25;
const int moveLeftPin      = 14;
const int moveRightPin     = 12;

unsigned long startTime;

void setup() {
  Serial.begin(115200);
 //startTime = millis();

  //Header CSV
 // Serial.println("Voltage1,Voltage2,Voltage3");
}

void loop() {
 
 //unsigned long currentTime = millis() - startTime;

  // Letture raw
  int rawValue1 = analogRead(hallPin1);
  int rawValue2 = analogRead(hallPin2);
  int rawValue3 = analogRead(hallPin3);


  float voltage1 = rawValue1 * (3.3 / 4095.0);
  float voltage2 = rawValue2 * (3.3 / 4095.0);
  float voltage3 = rawValue3 * (3.3 / 4095.0);

  
 //Serial.print(currentTime);
//Serial.print(",");
  Serial.print(voltage1, 3);
  Serial.print(",");
  Serial.print(voltage2, 3);
  Serial.print(",");
  Serial.println(voltage3, 3);

  delay(10); 
}

