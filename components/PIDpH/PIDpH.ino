/************************pH Sensor***************************/
#define phSensorPin         A0    //pH meter Analog output to Arduino Analog Input 0
#define VREF                5.0
#define phOffset            0.42  //deviation compensate
#define phNoSamples         20     //times of collection
float phTotal = 0;
float phAverage = 0;
float phValue, phVoltage;

/************************Water Pump**************************/
#define waterPumpPin 6

/************************PID Variables**************************/
double kp = -1000;
double ki = -2;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError;
double input, output, setPoint;
double cummulativeError, rateError;
int outMin, outMax;

float pHValues[50], outputValues[50], errorValues[50];
int counter = 0;

void setup() {
  //PID
  setPoint = 6.00;

  //Water Pump
  pinMode(waterPumpPin, OUTPUT);
  digitalWrite(waterPumpPin, HIGH);

  Serial.begin(115200);
}

void loop() {
  pH();
  input = phValue;
  Serial.println(phValue);

  if (phValue < 5.5) {
    Serial.println("pH is too low! Change the water!");
  } else if (phValue > 6.0) {
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
    
    pHValues[counter] = input;
    outputValues[counter] = output;
    errorValues[counter] = error;
    counter++;

    digitalWrite(waterPumpPin, LOW);
    delay(output);
    digitalWrite(waterPumpPin, HIGH);

    delay(10000); // wait for pH to stabilised
  } else {
    Serial.println("pH is ok!");
    for (int i = 0; i <= counter; i++) {
      Serial.print(String(pHValues[i]) + " ");
    }
    Serial.println();
    for (int i = 0; i <= counter; i++) {
      Serial.print(String(outputValues[i]) + " ");
    }
    Serial.println();
    for (int i = 0; i <= counter; i++) {
      Serial.print(String(errorValues[i]) + " ");
    }
  }
}

double computePID(double input) {
  currentTime = millis(); //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation
  elapsedTime = elapsedTime/1000; // ms -> s
  error = setPoint - input; // determine error
  cummulativeError += error * elapsedTime; // compute integral
  rateError = (error - lastError)/elapsedTime; // compute derivative

  double out = kp * error + ki * cummulativeError + kd * rateError; //PID output

  lastError = error; //remember current error
  previousTime = currentTime; //remember current time

  if (out < 0) out = 0;
  
  return out; //have function return the PID output
}

void pH() {
  for (int thisReading = 0; thisReading < phNoSamples; thisReading++) {
    phTotal = phTotal + analogRead(phSensorPin);
    delay(5);
  }

  // calculate the average:
  phAverage = phTotal / phNoSamples;
  phTotal = 0;
  phVoltage = phAverage * VREF / 1023;
  phValue = 3.5 * phVoltage + phOffset;
}
