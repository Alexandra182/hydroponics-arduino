#define TDSSensorPin         A1   //TDS meter Analog output to Arduino Analog Input 0
#define VREF                 5.0  //analog reference voltage(Volt) of the ADC
#define TDSNoSamples         30   //times of collection
#define TDSPrintInterval     100

float temperature = 25;
float TDSVoltage, TDSCompensationCoefficient, TDSCompensationVoltage;
int TDSSamples[TDSNoSamples];        //Store the average value of the sensor feedback
int TDSValue, TDSTotal, TDSAverage;
int TDSSamplesIndex = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("TDS meter experiment!");
  pinMode(TDSSensorPin, INPUT);

  // initialising all the readings to 0:
  for (int thisReading = 0; thisReading < TDSNoSamples; thisReading++) {
    TDSSamples[thisReading] = 0;
  }
}

void loop()
{
  static unsigned long TDSPrintTime = millis();
  if (millis() - TDSPrintTime > TDSPrintInterval)  //every 40 milliseconds,read the analog value from the ADC
  {
    TDS();
    Serial.println(TDSValue);
    TDSPrintTime = millis();
  }
}

void TDS() {
  // subtract the last reading/reset the array:
  TDSTotal = TDSTotal - TDSSamples[TDSSamplesIndex];
  // read from the sensor:
  TDSSamples[TDSSamplesIndex] = analogRead(TDSSensorPin);
  // add the reading to the total:
  TDSTotal = TDSTotal + TDSSamples[TDSSamplesIndex];
  // advance to the next position in the array:
  TDSSamplesIndex = TDSSamplesIndex + 1;

  // if the end of the array is reached
  if (TDSSamplesIndex >= TDSNoSamples) {
    TDSSamplesIndex = 0;
  }

  // calculate the average:
  TDSAverage = TDSTotal / TDSNoSamples;
  TDSVoltage = TDSAverage * VREF / 1023.0;
  //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  TDSCompensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); 
  TDSCompensationVoltage = TDSVoltage / TDSCompensationCoefficient; //temperature compensation
  TDSValue = (133.42 * pow(TDSCompensationVoltage,3) - 255.86 * pow(TDSCompensationVoltage, 2) + 857.39 * TDSCompensationVoltage) * 0.5;
}
