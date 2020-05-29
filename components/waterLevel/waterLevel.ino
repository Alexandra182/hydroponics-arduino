#define waterlvlPin A2

void setup() {
  pinMode(waterlvlPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int waterLevel = analogRead(waterlvlPin); 
  Serial.print("Water level: ");
  Serial.println(waterLevel);
  delay(1000);
}
