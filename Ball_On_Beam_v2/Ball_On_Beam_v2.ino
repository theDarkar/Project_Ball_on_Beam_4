#include <Servo.h>

Servo mainServo;                        // Create servo object to control a servo

const boolean LOGGING = true;

const double MIN_SERVO_ANGLE = 82;    // The minimum angle the servo can reach
const double MAX_SERVO_ANGLE = 148;   // The maximum angle the servo can reach
const double MIN_BEAM_ANGLE = 0;      // The minimum angle the beam is allowed to travel
const double MAX_BEAM_ANGLE = 1023;   // The minimum angle the beam is allowed to travel
const float MIN_BALL_DISTANCE = 10.00; // The minimum distance the ball is allowed to travel
const float MAX_BALL_DISTANCE = 35.00; // The maximum distance the ball is allowed to travel
const double K_P = 1;                 // The proportional gain
const double K_I = 0;                 // The integral gain
const double K_D = 0;                 // The derivative gain
const int MANUAL_MODE = 0;            // State for manual mode
const int REG_MODE = 1;               // State for reguleting mode
const int LED_PIN = 13;               // Pin connected to LED
const int BUTTON_PIN = 8;             // Pin connected to red buttton
const int POT_PIN = A0;               // Pin connected to potmeter
const int DIST_SENSOR_PIN = A1;       // Pin connected to distance sensor
const int SERVO_PIN = 10;              // Pin connected to servo
const int MAX_VOLTAGE = 5;            // Maximum voltage the analog pin can read
const int INITIAL_SERVO_POS = 115;    // Initial servo position
const int NUM_READINGS = 10;          // The amount of readings taken before given an average

int INPUT_TABLE[] = {620, 594.5, 569, 549, 529, 505.5, 482, 464.5, 447, 429, 411, 397.5, 384, 373, 362, 351.5, 341, 331.5, 322, 314, 306, 299.5, 293, 285, 277, 273.5, 270, 263.5, 257, 251.5, 246, 242, 238, 233, 228, 223.5, 219, 217.5, 216, 213, 210, 207, 204, 203, 202, 199.5, 197, 193.5, 190, 188, 186, 183.5, 181, 180.5, 180, 178.5, 177, 174.5, 172, 170.5, 169, 167.5, 166, 164, 162};
int INPUT_INDEX[] = {7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17, 17.5, 18, 18.5, 19, 19.5, 20, 20.5, 21, 21.5, 22, 22.5, 23, 23.5, 24, 24.5, 25, 25.5, 26, 26.5, 27, 27.5, 28, 28.5, 29, 29.5, 30, 30.5, 31, 31.5, 32, 32.5, 33, 33.5, 34, 34.5, 35, 35.5, 36, 36.5, 37, 37.5, 38, 38.5, 39};

float beamAngle = 0;            // Angle of the beam
float ballDistance = 0;        // Distance of ball
int currentMode = MANUAL_MODE;  // Sets the current operational state
int servoAngle = 0;             // Angle of the servoinput
int readIndex = 0;              // the index of the current reading
int totalDist = 0;              // the running total
int lastError = 0;
int integral = 0;
double PID = 0;
int val = 0;
int set = 0;
int ball = 0;
double newBeamAngle = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  mainServo.attach(SERVO_PIN);  // attaches the servo on pin 10 to the servo object
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
      regMode(POT_PIN, ballDistance);
      if (!isButtonPressed(BUTTON_PIN)) {
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
void regMode(int potPin, float ballDist) {
  ball = map(ballDist, 580, 130, 0, 1023);
  set = getPotmeterReading(potPin);
  int error = set - ball;
  double P = proportionalValue(0.2, error);
  double I = 0;//integralValue(1, error);
  double D = derivativeValue(1, error, lastError);
  PID = P + I + D;
  int currentBeamAngle = analogRead(SERVO_PIN); 
  newBeamAngle = currentBeamAngle+PID;
  setBeamAngle(map(newBeamAngle,0,1023,180,0));
  lastError = error;
}

/**
   Calculates the P value in the PID-regulator

   @param err The error value
   @param Kp The proportional gain
   @return The P value
*/
double proportionalValue(double Kp, int err) {
  double P = Kp * err;
  return P;
}

/**
   Calculates the I value in the PID-regulator

   @param err The error value
   @param Ki The integral gain
   @return The I value
*/
int integralValue(double Ki,int err) {
  int I = Ki * (integral + err);
  integral = I;
  return I;
}

/**
   Calculates the D value in the PID-regulator

   @param Kd The derivative gain
   @param err The error value
   @param oldErr The old error value
   @return The D value
*/
double derivativeValue(double Kd, int err, int oldErr) {
  double D = Kd * (err - oldErr);
  return D;
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
    delay(5);
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
  Serial.print(analogRead(SERVO_PIN));
  //Serial.print(servoAng);
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
  Serial.print(ballDist);
  Serial.print(" / ");
  Serial.print(map(ballDist, 560, 140, 0, 1023));
  Serial.print("   ");
  Serial.print("  Servo angle: ");
  Serial.print(servoAng);
  Serial.print(" / ");
  Serial.print(map(servoAng, 0, 180, 0, 1023));
  Serial.print("  New beam angle: ");
  Serial.print(newBeamAngle);
  Serial.print("  Error: ");
  Serial.print(lastError); 
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
      Serial.print(map(set,0,1023,0,42));
      Serial.print(" cm / ");
      Serial.print(set);
      break;
    default:
      Serial.print("!!!ERROR!!!");
      break;
    }
}
  Serial.println("");
}
