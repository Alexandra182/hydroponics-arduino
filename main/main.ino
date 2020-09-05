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

/****************************PID*****************************/
double kp = -1000;
double ki = -2;
double kd = -100;

unsigned long currentTime, previousTime, PIDstartTime, phStabiliseTime;
double elapsedTime;
double error, lastError;
double input, output;
double setPoint = 6.00; // ideal pH
double cummulativeError, rateError;
int outMin, outMax;

void setup() {
  //Serial Communication
  Serial.begin(115200);

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
      pH();
      TDS();
      waterLvl();
      measurement = String(phValue) + "-" + String(TDSValue) + "-" + String(int(temperature)) + "-" + String(int(humidity)) + "-" + String(waterLevel);
      Serial.println(measurement); // Send data to RaspberryPi
    }
  }

  pH();
  input = phValue;

  if (phValue < 5.5) {
    //Serial.println("pH is too low! Change the water!");
  } else if (phValue > 6.0) { // Use PID to adjust the pH
    phStabiliseTime = millis() - PIDstartTime;
    if (phStabiliseTime >= 20000) { // if 10s have passed the pH is stable
      output = computePID(input);

      /*
      Serial.println("Input: " + String(input));
      Serial.println("Output: " + String(output));
      Serial.println("Setpoint: " + String(setPoint));
      Serial.println("Error: " + String(error));
      Serial.println("Cummulative Error: " + String(cummulativeError));
      Serial.println("Rate Error: " + String(rateError));
      Serial.println("Elapsed Time: " + String(elapsedTime));
      Serial.println();
      */

      digitalWrite(waterPumpPin, LOW);
      timer.setTimeout(output, waterPumpOff);

      PIDstartTime = millis(); // wait for pH to stabilised
    }
  } else {
    //Serial.println("pH is ok!");
  }
}

void TemperatureHumidity() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again)
  if (isnan(humidity) || isnan(temperature)) {
    // Serial.println(F("Failed to read from the DHT sensor!"));
    temperature = 25;
    humidity = 50;
    return;
  }
}

void pH() {
  for (int thisReading = 0; thisReading < phNoSamples; thisReading++) {
    phTotal = phTotal + analogRead(phSensorPin);
  }

  // calculate the average:
  phAverage = phTotal / phNoSamples;
  phTotal = 0;
  phVoltage = phAverage * VREF / 1023;
  phValue = 3.5 * phVoltage + phOffset;
}

void TDS() {
  for (int thisReading = 0; thisReading < TDSNoSamples; thisReading++) {
    TDSTotal = TDSTotal + analogRead(TDSSensorPin);
  }

  // calculate the average:
  TDSAverage = TDSTotal / TDSNoSamples;
  TDSTotal = 0;
  TDSVoltage = TDSAverage * VREF / 1023.0;
  // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  TDSCompensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  TDSCompensationVoltage = TDSVoltage / TDSCompensationCoefficient; // temperature compensation
  TDSValue = (133.42 * pow(TDSCompensationVoltage, 3) - 255.86 * pow(TDSCompensationVoltage, 2) + 857.39 * TDSCompensationVoltage) * 0.5;
}

void waterLvl() {
  waterLevel = analogRead(waterlvlPin);
  // map the analog values to percentages
  waterLevel = map(waterLevel, 0, 600, 0, 100);
}

void airPump() {
  if (onState) {
    digitalWrite(airPumpPin, LOW); // turn on
  } else {
    digitalWrite(airPumpPin, HIGH); // turn off
  }
  onState = !onState;
}

double computePID(double input) {
  currentTime = millis(); // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation
  elapsedTime = elapsedTime / 1000; // ms -> s
  error = setPoint - input; // determine error
  cummulativeError += error * elapsedTime; // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cummulativeError + kd * rateError; // PID output

  lastError = error; // remember current error
  previousTime = currentTime; // remember current time

  if (out < 0) out = 0;

  return out; // have function return the PID output
}

void waterPumpOff() {
  digitalWrite(waterPumpPin, HIGH);
}
