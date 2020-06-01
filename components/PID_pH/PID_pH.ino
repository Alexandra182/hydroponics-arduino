/************************pH Sensor***************************/
#define phSensorPin         A0    //pH meter Analog output to Arduino Analog Input 0
#define VREF                5.0
#define phOffset            0.42  //deviation compensate
#define phNoSamples         40     //times of collection
float phTotal = 0;
float phAverage = 0;
float phValue, phVoltage;

/************************Water Pump**************************/
#define waterPumpPin 6

/************************PID Variables**************************/
double kp = -700;
double ki = 0;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError;
double input, output, setPoint;
double cummulativeError, rateError;

float pHValues[100], outputValues[100], errorValues[100];
int counter = 0;

void setup() {
  //PID
  setPoint = 6.00;

  //Water Pump
  pinMode(waterPumpPin, OUTPUT);
  digitalWrite(waterPumpPin, HIGH);

  Serial.begin(9600);
}

void loop()
{
  pH();
  input = phValue;

  if (phValue < 5.5) {
    Serial.println("pH is too low! Change the water!");
  } else if (phValue > 6.5) {
    output = computePID(input);

    Serial.println("Input: " + String(input));
    Serial.println("Output: " + String(output));
    Serial.println("Setpoint: " + String(setPoint));
    Serial.println("Error" + String(error));
    Serial.println();

    pHValues[counter] = input;
    outputValues[counter] = output;
    errorValues[counter] = error;
    counter++;

    digitalWrite(waterPumpPin, LOW);
    delay(output);
    digitalWrite(waterPumpPin, HIGH);

    delay(5000); // wait for pH to stabilised
  } else {
    Serial.println("pH is ok!");
    for (int i = 1; i++; i <= counter) {
      Serial.println(pHValues[i]);
      Serial.println(outputValues[i]);
      Serial.println(errorValues[i]);
    }
  }
}

double computePID(double input) {
  currentTime = millis(); //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

  error = setPoint - input; // determine error
  cummulativeError += error * elapsedTime; // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cummulativeError + kd * rateError; //PID output

  lastError = error; //remember current error
  previousTime = currentTime; //remember current time

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
  phVoltage = phAverage * VREF / 1024;
  phValue = 3.5 * phVoltage + phOffset;
}
