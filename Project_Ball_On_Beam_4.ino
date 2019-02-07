#include <Servo.h>
#include <SharpIR.h>

Servo mainServo;                        // Create servo object to control a servo
SharpIR distSensor("GP2Y0A21YK0F", A1); // Create sensor object to get distance of the ball

const double BALL_WEIGHT = 10;        // Weight of ball
const double ARM_RADIUS = 10;         // Radius of the arm on the wheel
const double ARM_LENGTH = 10;         // Length of the arm between the beam and the wheel
const double MIN_SERVO_ANGLE = 0;     // The minimum angle the servo can reach
const double MAX_SERVO_ANGLE = 180;   // The maximum angle the servo can reach
const double MIN_BEAM_ANGLE = -10;    // The minimum angle the beam is allowed to travel
const double MAX_BEAM_ANGLE = 10;     // The minimum angle the beam is allowed to travel
const double BALL_MIN_DISTANCE = 100; // The minimum distance the ball is allowed to travel
const double BALL_MAX_DISTANCE = 500; // The maximum distance the ball is allowed to travel
const int MANUAL_MODE = 0;            // State for manual mode
const int REG_MODE = 1;               // State for reguleting mode
const int LED_PIN = 13;               // Pin connected to LED
const int RED_BUTTON_PIN = 8;         // Pin connected to red buttton
const int POT_PIN = A0;               // Pin connected to potmeter
const int SERVO_PIN = 9;              // Pin connected to servo
const int MAX_VOLTAGE = 5;            // Maximum voltage the analog pin can read
const int INITIAL_SERVO_POS = 90;     // Initial servo position

int servoAngle = 0;            // Angle of the servo
float beamAngle = 0;           // Angle of the beam
float ballDistance = 0;        // Distance of ball
int currentMode = MANUAL_MODE; // Sets the current opartional state

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_BUTTON_PIN, INPUT);
  mainServo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  setServoPos(INITIAL_SERVO_POS);
  Serial.begin(9600);
}

void loop() {
  servoAngle = getDesiredServoPos(SERVO_PIN); 
  beamAngle = getBeamAngle(servoAngle);
  ballDistance = getBallPos(true);
  setLedPins(currentMode, LED_PIN);
  switch (currentMode) {
    case MANUAL_MODE:
      manualMode(POT_PIN);
      if (isButtonPressed(RED_BUTTON_PIN)) {
        currentMode = REG_MODE;
      }
      break;
    case REG_MODE:
      regMode(POT_PIN, ballDistance, beamAngle);
      if (isButtonPressed(RED_BUTTON_PIN)) {
        currentMode = MANUAL_MODE;
      }
      break;
    default:
      break;
  }
  printToSerial(currentMode, ballDistance, beamAngle, servoAngle, POT_PIN);
}

/**
   Reads the position of the potmeter and adjust the 
   angle of the beam angle thereafter

   @param potPin The pin the potmeter is connected to
*/
void manualMode(int potPin) {
  float newBeamAngle = map(getPotmeterReading(potPin), 0, 1023, MIN_BEAM_ANGLE , MAX_BEAM_ANGLE);
  //setBeamAngle(newBeamAngle);
}

/**
   Regulates the beam angle so the ball is at the 
   setpoint givven by the potmeter

   @param potPin The pin the potmeter is connected to
   @param ballDist The current ball distance
   @param beamAngle The current beam angle
*/
void regMode(int potPin, float ballDist, float beamAng) {
  float setpoint = map(getPotmeterReading(potPin), 0, 1023, BALL_MIN_DISTANCE, BALL_MAX_DISTANCE);
  float P = proportionalValue(setpoint, ballDist);
  float I = integralValue(setpoint, ballDist);
  float D = derivativeValue(setpoint, ballDist);
  float newBeamAngle = P + I + D;
  setBeamAngle(newBeamAngle);
}

/**
   Calculates the P value in the PID-regulator

   @param setpoint The desired value
   @param currentValue The current value
   @return The P value
*/
float proportionalValue(double setpoint, double currentValue) {
  float P = 0;
  return P;
}

/**
   Calculates the I value in the PID-regulator

   @param setpoint The desired value
   @param currentValue The current value
   @return The I value
*/
float integralValue(double setpoint, double currentValue) {
  float I = 0;
  return 0;
}

/**
   Calculates the D value in the PID-regulator

   @param setpoint The desired value
   @param currentValue The current value
   @return The D value
*/
float derivativeValue(double setpoint, double currentValue) {
  float D = 0;
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

   @param avoidBurstRead When set to true will give readings on demand,
                         when set to false the readings will come only after 20 ms
   @return Returns the current ball position in mm
*/
float getBallPos(boolean avoidBurstRead) {
  byte dist = distSensor.getDistance(avoidBurstRead); // Gets the distance in cm
  float ballDist = dist * 10;
  return ballDist;
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
  int potPos = map(getVoltage(pin), 0, MAX_VOLTAGE, 0, 1023);
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
   Sets the angle of the beam

   @param angle The desired angle of the beam
*/
void setBeamAngle(float angle) {
  int pos = map(angle, MIN_BEAM_ANGLE, MAX_BEAM_ANGLE, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
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
  cmdPos = map(cmdPos, 0, 1023, 0, 180);  // Scale the input to a value between 0 and 180
  return cmdPos;
}

/**
   Checks if the button pin is HIGH or LOW and returns true if the
   button ispressed or false if the button isn't pressed

   @param buttonPin The pin the button is connected to
   @return True if button is pressed, flase if not
*/
boolean isButtonPressed(int buttonPin)
{
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
void turnLedOn(int ledPin)
{
  digitalWrite(ledPin, HIGH);
}

/**
   Turns LED connected to declared pin off

   @param ledPin The pin connected to the LED
*/
void turnLedOff(int ledPin)
{
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
  Serial.print("Ball distance: ");
  Serial.print(ballDist);
  Serial.print("  Beam angle: ");
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
  }
  Serial.println("");
}
