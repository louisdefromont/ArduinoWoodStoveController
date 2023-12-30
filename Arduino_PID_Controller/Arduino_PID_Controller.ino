/*    Max6675 Module  ==>   Arduino
 *    CS              ==>     D11
 *    SO              ==>     D12
 *    SCK             ==>     D13
 *    Vcc             ==>     Vcc (5v)
 *    Gnd             ==>     Gnd      */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <Stepper.h>

// Constants to calibrate the thermocouple readings
// Use freezing water and boiling water to adjust the constants
// Set calibrateModeOn to true while callibrating, then false to use calibrations
const double calibrate0 = 5.00; // The measured temperature reading at 0 c
const double calibrate100 = 100.00; // The measured temperature reading at 100 c
const bool calibrateModeOn = false; // Set to true to get raw temperature readings in order to calibrate

#define MAX6675_CS   11
#define MAX6675_SO   12
#define MAX6675_SCK  13

const int stepsPerRevolution = 2048;        // change this to fit the number of steps per revolution
const int rolePerMinute = 17;               // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
const int temperatureReadInterval = 1000;    // Number of ms between temperature reads
const int targetMotorPositionInterval = 10000;
const double upperStepsBound = 81920;       // Number of revolutions needed to fully close the valve
const float autoTemperatureDeadzone = 5.0;
const double stepsPerMotorTurn = 100;

SoftwareSerial softSerial(2, 3);
Stepper stepper(stepsPerRevolution, 7, 9, 8, 10);

double targetSteps = 0;
double currentSteps = 0;
unsigned long lastTemperatureRead = 0;
unsigned long lastTargetMotorPosition = 0;
float currentTemperature = 0.0;
float targetTemperature = 240.0;
bool automaticTemperatureControl = false;
int manualMotorDirection = 0;

void setup() {
  Serial.begin(9600);
  softSerial.begin(115200);
  stepper.setSpeed(rolePerMinute);
}

void loop() {
  readAndTransmitTemperature();
  readESP8266();
  if (automaticTemperatureControl) {
    setTargetMotorPosition();
  }
  turnMotor();
  if (manualMotorDirection != 0) {
    turnMotorManual(manualMotorDirection);
  }
}

void readAndTransmitTemperature() {
  unsigned long currentTime = millis();
  if (currentTime - lastTemperatureRead >= temperatureReadInterval) {
    lastTemperatureRead = currentTime;
    currentTemperature = readThermocouple();
    softSerial.println(currentTemperature);
  }
}

void readESP8266() {
  if (softSerial.available()) {
    String serialInput = softSerial.readString();
    Serial.print("SoftSerial//" + serialInput);
    
    if (serialInput.startsWith("T")) {
      targetTemperature = serialInput.substring(1).toFloat();
      automaticTemperatureControl = true;
      Serial.println("Set target temperature to: " + String(targetTemperature));
    } else if (serialInput.startsWith("M")) {
      double target = serialInput.substring(1).toDouble();
      if (target < 0) {
        setTargetSteps(currentSteps);
        manualMotorDirection = 0;
      } else {
        setTargetSteps((target / 100.0) * double(upperStepsBound));
      }
      automaticTemperatureControl = false;
    } else if (serialInput.startsWith("D")) {
      int direction = serialInput.substring(1).toInt();
      manualMotorDirection = direction;
      automaticTemperatureControl = false;
    }
    
  }
}

void setTargetMotorPosition() {
  unsigned long currentTime = millis();
  if (currentTime - lastTargetMotorPosition >= targetMotorPositionInterval) {
    lastTargetMotorPosition = currentTime;
    if (currentTemperature - autoTemperatureDeadzone > targetTemperature) {
      setTargetSteps(currentSteps - 1);
    } else if (currentTemperature + autoTemperatureDeadzone < targetTemperature) {
      setTargetSteps(currentSteps + 1);
    }
  }
}

void turnMotor() {
  if (targetSteps == currentSteps) {
    return;
  }
  int direction = (targetSteps > currentSteps) ? 1 : -1;
  double stepsToTake = min(abs(targetSteps - currentSteps), stepsPerMotorTurn);
  stepsToTake = stepsToTake * direction;
  stepper.step(stepsToTake);
  currentSteps += stepsToTake;
}

void turnMotorManual(int direction) {
  stepper.step(direction * stepsPerMotorTurn);
}

void setTargetSteps(double ts) {
  if (ts > upperStepsBound) {
    targetSteps = upperStepsBound;
  } else if (ts < 0) {
    targetSteps = 0;
  } else {
    targetSteps = ts;
  }
}

double readThermocouple() {
  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  double temperature = v*0.25;
  
  if (calibrateModeOn) {
    return temperature; // Return raw temperature readings if callibrateModeOn is set to true
  } else {
    return (temperature - calibrate0) * (100.0 / (calibrate100 - calibrate0));
  }
}