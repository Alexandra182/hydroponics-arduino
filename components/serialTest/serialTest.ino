int ph = 4.3;
int tds = 200;
int temp = 25;
int humidity = 32;
int waterlvl = 100;
String measurement;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "S") {
      measurement = String(ph) + "-" + String(tds) + "-" + String(temp) + "-" + String(humidity) + "-" + String(waterlvl);
      Serial.println(measurement);
    }
  }
}
