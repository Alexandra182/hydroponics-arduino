/************************pH Sensor***************************/
#define phSensorPin         A0    //pH meter Analog output to Arduino Analog Input 0
#define VREF                5.0
#define phOffset            -0.15  //deviation compensate
#define phNoSamples         40     //times of collection
float phTotal = 0;
float phAverage = 0;
float phValue, phVoltage;

/************************TDS Sensor**************************/
#define TDSSensorPin         A1   //TDS meter Analog output to Arduino Analog Input 0
#define VREF                 5.0  //analog reference voltage(Volt) of the ADC
#define TDSNoSamples         30   //times of collection
float TDSVoltage, TDSCompensationCoefficient, TDSCompensationVoltage;
int TDSValue, TDSTotal, TDSAverage;

/************************Temperature*************************/
#include "DHT.h"
#define DHTPin 4     // Digital pin connected to the DHT sensor
#define DHTType DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPin, DHTType);
float humidity, temperature;

/************************Water Level*************************/
#define waterlvlPin A2
int waterLevel;

/************************Water Pump**************************/
#define waterPumpPin 6

/*************************Air Pump***************************/
#include "SimpleTimer.h"
#define airPumpPin 7
SimpleTimer timer;
bool onState = false;

/*********************RPI Communication**********************/
String measurement;

void setup() {
  //Serial Communication
  Serial.begin(9600);

  // Sensors
  pinMode(TDSSensorPin, INPUT);
  pinMode(phSensorPin, INPUT);
  pinMode(waterlvlPin, INPUT);
  dht.begin();

  // Actuators
  pinMode(waterPumpPin, OUTPUT);
  pinMode(airPumpPin, OUTPUT);

  digitalWrite(waterPumpPin, HIGH);
  digitalWrite(airPumpPin, HIGH);

  // Air Pump Timer
  timer.setInterval(3000, airPump);
}

void loop() {
  timer.run();
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "S") {
      TemperatureHumidity();
      temperature = (random(22, 27));
      humidity = (random(70, 80));
      phValue = float(random(500, 720)) / 100;
      TDSValue = random(300, 440);
      waterLevel = random(0, 100);
      measurement = String(phValue) + "-" + String(TDSValue) + "-" + String(int(temperature)) + "-" + String(int(humidity)) + "-" + String(waterLevel);
      Serial.println(measurement);
    }
  }
  //waterPump();
  //delay(1000);
  //airPump();
  //delay(1000);
}

void TemperatureHumidity() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    //Serial.println(F("Failed to read from DHT sensor!"));
    temperature = 0;
    humidity = 0;
    return;
  }
}

void pH() {
  for (int thisReading = 0; thisReading < phNoSamples; thisReading++) {
    phTotal = phTotal + analogRead(phSensorPin);
    delay(5);
  }

  // calculate the average:
  phAverage = phTotal / phNoSamples;
  phTotal = 0;
  phVoltage = phAverage * VREF / 1024;
  phValue = 3.5 * phVoltage + phOffset;
}

void TDS() {
  for (int thisReading = 0; thisReading < TDSNoSamples; thisReading++) {
    TDSTotal = TDSTotal + analogRead(TDSSensorPin);
    delay(5);
  }

  // calculate the average:
  TDSAverage = TDSTotal / TDSNoSamples;
  TDSTotal = 0;
  TDSVoltage = TDSAverage * VREF / 1024.0;
  //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  TDSCompensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  TDSCompensationVoltage = TDSVoltage / TDSCompensationCoefficient; //temperature compensation
  TDSValue = (133.42 * pow(TDSCompensationVoltage, 3) - 255.86 * pow(TDSCompensationVoltage, 2) + 857.39 * TDSCompensationVoltage) * 0.5;
}

void waterLvl() {
  waterLevel = analogRead(waterlvlPin);
  // map the analogue values to percentages
  waterLevel = map(waterLevel, 0, 600, 0, 100);
}

void waterPump() {
  digitalWrite(waterPumpPin, LOW);
  delay(1000);
  digitalWrite(waterPumpPin, HIGH);
}

void airPump() {
  if (onState) {
    //digitalWrite(airPumpPin, LOW); // turn on
    digitalWrite(airPumpPin, HIGH); // DE STERS DUPA CE TERMIN
    Serial.println("LOW");
  } else {
    digitalWrite(airPumpPin, HIGH); // turn off
    Serial.println("HIGH");
  }
  onState = !onState;
}
