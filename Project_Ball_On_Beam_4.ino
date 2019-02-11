#include <Servo.h>

Servo mainServo;                        // Create servo object to control a servo

const boolean LOGGING = true;
const double BALL_WEIGHT = 10;        // Weight of ball
const double ARM_RADIUS = 10;         // Radius of the arm on the wheel
const double ARM_LENGTH = 10;         // Length of the arm between the beam and the wheel
const double MIN_SERVO_ANGLE = 82;    // The minimum angle the servo can reach
const double MAX_SERVO_ANGLE = 148;   // The maximum angle the servo can reach
const double MIN_BEAM_ANGLE = 0;      // The minimum angle the beam is allowed to travel
const double MAX_BEAM_ANGLE = 1023;   // The minimum angle the beam is allowed to travel
const double BALL_MIN_DISTANCE = 100; // The minimum distance the ball is allowed to travel
const double BALL_MAX_DISTANCE = 500; // The maximum distance the ball is allowed to travel
const double K_P = 1;                 // The proportional gain
const double K_I = 0;                 // The integral gain
const double K_D = 0;                 // The derivative gain
const int MANUAL_MODE = 0;            // State for manual mode
const int REG_MODE = 1;               // State for reguleting mode
const int LED_PIN = 13;               // Pin connected to LED
const int BUTTON_PIN = 8;             // Pin connected to red buttton
const int POT_PIN = A0;               // Pin connected to potmeter
const int DIST_SENSOR_PIN = A1;       // Pin connected to distance sensor
const int SERVO_PIN = 9;              // Pin connected to servo
const int MAX_VOLTAGE = 5;            // Maximum voltage the analog pin can read
const int INITIAL_SERVO_POS = 115;    // Initial servo position
const int NUM_READINGS = 100;         // The amount of readings taken before given an average

int INPUT_TABLE[] = {620,569,529,482,447,411,384,362,341,322,306,293,277,270,257,246,238,228,219,216,210,204,202,197,190,186,181,180,177,172,169,166,162};
int INPUT_INDEX[] = {7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39};

float beamAngle = 0;            // Angle of the beam
double ballDistance = 0;        // Distance of ball
double oldBallDistance = 0;     // Old distance of ball
int currentMode = MANUAL_MODE;  // Sets the current operational state
int servoAngle = 0;             // Angle of the servoinput
int readIndex = 0;              // the index of the current reading
int totalDist = 0;              // the running total
int averageDist = 0;            // 
int angle1 = 0;
int angle2 = 0;
boolean a = true;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  mainServo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  setServoPos(INITIAL_SERVO_POS);
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  beamAngle = getBeamAngle(servoAngle);
  ballDistance = getBallPos(NUM_READINGS,INPUT_TABLE,INPUT_INDEX,DIST_SENSOR_PIN);
  setLedPins(currentMode, LED_PIN);
  
  switch (currentMode) {
    case MANUAL_MODE:
      manualMode(POT_PIN);
      if (!isButtonPressed(BUTTON_PIN)) {
        currentMode = REG_MODE;
      }
      break;
    case REG_MODE:
      regMode(POT_PIN, ballDistance, oldBallDistance);
      if (!isButtonPressed(BUTTON_PIN)) {
        currentMode = MANUAL_MODE;
      }
      break;
    default:
      break;
  }
  printToSerial(currentMode, ballDistance, beamAngle, servoAngle, POT_PIN);
  oldBallDistance = ballDistance;
}

/**
   Reads the position of the potmeter and adjust the
   angle of the beam angle thereafter

   @param potPin The pin the potmeter is connected to
*/
void manualMode(int potPin) {
  int pos = getDesiredServoPos(potPin);
  setBeamAngle(pos);
}

/**
   Regulates the beam angle so the ball is at the
   setpoint given by the potmeter

   @param potPin The pin the potmeter is connected to
   @param ballDist The current ball distance
   @param beamAngle The current beam angle
*/
void regMode(int potPin, float ballDist, oldBallDist) {
  int setpoint = getPotmeterReading(potPin);//= map(getPotmeterReading(potPin), 0, 1023, BALL_MIN_DISTANCE, BALL_MAX_DISTANCE);
  int error = setpoint - currentValue;
  int P = proportionalValue(K_P, error);
  int I = integralValue(K_I, error, ballDist);
  int D = derivativeValue(K_D, error, lastError);
  int newBeamAngle = getDesiredServoPos(P + I + D);
  setBeamAngle(newBeamAngle);
  lastError = error;
}

/**
   Calculates the P value in the PID-regulator

   @param setpoint The desired value
   @param currentValue The current value
   @param Kp The proportional gain
   @return The P value
*/
int proportionalValue(double Kp, int err) {
  int P = Kp * err;
  return P;
}

/**
   Calculates the I value in the PID-regulator

   @param setpoint The desired value
   @param currentValue The current value
   @param Ki The integral gain
   @return The I value
*/
int integralValue(double Ki, double setpoint, double currentValue) {
  int I = 0;
  return 0;
}

/**
   Calculates the D value in the PID-regulator

   @param Kd The derivative gain
   @param currentValue The current value
   @param oldValue The old value
   @return The D value
*/
int derivativeValue(double Kd, int err, oldErr) {
  int D = Kd*(err - oldErr);
  return 0;
}

/**
   Gets the angle of the beam

   @param servoAng
   @return Returns the current beam angle
*/
float getBeamAngle(int servoAng) {
  float beamAng = map(servoAng, MIN_SERVO_ANGLE , MAX_SERVO_ANGLE , -MIN_BEAM_ANGLE , MAX_BEAM_ANGLE );
  return beamAng;
}

/**
   Gets the ball position from distSensor
   TODO: average algorithim

   @param avoidBurstRead When set to true will give readings on demand,
                         when set to false the readings will come only after 20 ms
   @return Returns the current ball position in mm
*/
float getBallPos(int numReading, int inputTable[], int inputIndex[],int pin) {
  int distReadings[numReading];
  for (int i = 0; i < numReading - 1; i++) {
    distReadings[i] = analogRead(pin);
  }
  int sumReadings = 0;
  for(int i = 0; i < numReading -1; i++){
    sumReadings = sumReadings + distReadings[i];
  }
  int averageReading = sumReadings / numReading;
  return averageReading;
}

/**
   Sets the LED depending on the mode

   @param mode The current mode
   @param pin The pin of the LED to be used
*/
void setLedPins(int mode, int pin) {
  switch (mode) {
    case MANUAL_MODE:
      turnLedOff(pin);
      break;
    case REG_MODE:
      turnLedOn(pin);
      break;
    default:
      break;
  }
}

/**
   Reads the potmeter position

   @param The pin the potmeter is connected to
   @return The potmeter positon from 0 to 1023
*/
int getPotmeterReading(int pin) {
  // float potPos = map(getVoltage(pin), 0, MAX_VOLTAGE, 0, 1023);
  int potPos = analogRead(pin);
  return potPos;
}

/**
   Reads an analog pin and returns the current voltage

   @param pin The anlog pin to read the voltage from
   @return The current voltage on the pin
*/
float getVoltage(int pin) {
  return (analogRead(pin) * 0.004882814);
}

/**
   Sets the angle of the beam by a sending the angle to the servo 
   and constrain the angle from MIN_SERVO_ANGLE to MAX_SERVO_ANGLE.

   @param pos The desired servo pos 0 to 180
*/
void setBeamAngle(int pos) {
  pos = constrain(pos,MIN_SERVO_ANGLE,MAX_SERVO_ANGLE);
  setServoPos(pos);
}

/**
   Sets the position of the servo to the position
   given by the parameter pos

   @param pos The position to set the servo to in degrees.
              Must be between 0 and 180 degrees
*/
void setServoPos(int pos) {
  pos = constrain(pos, 0, 180);   // Make sure that pos is within the range 0 to 180
  servoAngle = pos;
  mainServo.write(pos);
}

/**
   Returns the desired position of the servo read
   from the potentiometer.
   The position is in degrees and is in the range 0 to 180 degrees.

   @param pin The pin that the potensiometer is connected to
   @returns The desired position to set the servo to in degrees
*/
int getDesiredServoPos(int pin) {
  int cmdPos = analogRead(pin);
  cmdPos = map(cmdPos, 0, 1023, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);  // Scale the input to a value between 0 and 180
  return cmdPos;
}

/**
   Checks if the button pin is HIGH or LOW and returns true if the
   button ispressed or false if the button isn't pressed

   @param buttonPin The pin the button is connected to
   @return True if button is pressed, false if not
*/
boolean isButtonPressed(int buttonPin) {
  boolean buttonStateTrueFalse;
  int buttonStateHighLow = digitalRead(buttonPin);
  if (buttonStateHighLow == HIGH)
  {
    buttonStateTrueFalse = true;
  } else
  {
    buttonStateTrueFalse = false;
  }
  return (buttonStateTrueFalse);
}

/**
   Turns LED connected to declared pin on

   @param ledPin The pin connected to the LED
*/
void turnLedOn(int ledPin) {
  digitalWrite(ledPin, HIGH);
}

/**
   Turns LED connected to declared pin off

   @param ledPin The pin connected to the LED
*/
void turnLedOff(int ledPin) {
  digitalWrite(ledPin, LOW);
}

/**
   Prints info to the serial monitor

   @param mode The current mode
   @param ballDist The ball distance
   @param beamAng The beams angle
   @param servoAng The seervo angle
   @param potPin The pin the potmeter is connected to
*/
void printToSerial(int mode, float ballDist, float beamAng, float servoAng, int potPin) {
 if(!LOGGING){
  Serial.print("Ball distance: ");
  Serial.print(ballDist);
  Serial.print("  ");
  Serial.print(ballDistance);
  Serial.print("cm  Beam angle: ");
  Serial.print(beamAng);
  Serial.print("  Servo angle: ");
  Serial.print(servoAng);
  Serial.print("  Mode: ");
  switch (mode) {
    case MANUAL_MODE:
      Serial.print("MANUAL");
      Serial.print("  Potmeter: ");
      Serial.print(getPotmeterReading(potPin));
      break;
    case REG_MODE:
      Serial.print("REGULATOR");
      Serial.print("  Setpoint: ");
      Serial.print(getPotmeterReading(potPin));
      break;
    default:
      Serial.print("!!!ERROR!!!");
      break;
  }}else{
  Serial.print("Ball distance: ");
  Serial.print(analogRead(A1));
  Serial.print("  Servo angle: ");
  Serial.print(servoAng);
  Serial.print("   ");
  Serial.print(angle1);
  Serial.print("   ");
  Serial.print("  Mode: ");
  switch (mode) {
    case MANUAL_MODE:
      Serial.print("MANUAL");
      Serial.print("  Potmeter: ");
      Serial.print(getPotmeterReading(potPin));
      break;
    case REG_MODE:
      Serial.print("REGULATOR");
      Serial.print("  Setpoint: ");
      Serial.print(getPotmeterReading(potPin));
      break;
    default:
      Serial.print("!!!ERROR!!!");
      break;
    }
}
  Serial.println("");
}
