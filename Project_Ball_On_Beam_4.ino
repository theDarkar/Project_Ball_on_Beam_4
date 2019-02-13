#include <Servo.h>

Servo mainServo;      // Create servo object to control a servo

// Settings
const int MANUAL_MODE = 1;
const int AUTO_MODE = 2;
int currentMode = AUTO_MODE;

// Physical limitations
const int MIN_SERVO_POS = 90;          // Lowest allowed servo position
const int MAX_SERVO_POS = 155;         // Highest allowed servo position
const int MID_SERVO_POS = 118;         // Middle / stable servo position
const int MIN_BALL_DISTANCE = 500;     // (analog value) the closest the ball is allowed to be
const int MAX_BALL_DISTANCE = 190;     // (analog value) the farthest the ball is allowed to be
// const float MIN_BALL_DISTANCE_CM = ???;  // (linearised) the closest the ball is allowed to be in centimetres
// const float MAX_BALL_DISTANCE_CM = ???;  // (linearised) the farthest the ball is allowed to be in centimetres

// PID
const float K_P = 0.2;    // The proportional gain
const float K_I = 0;      // The integral gain
const float K_D = 0.9;    // The derivative gain
float p = 0;              // The proportional output           
float i = 0;              // The integral output
float d = 0;              // The derivative output

// Log of distance readings
const int NUM_DIST_READINGS = 40;           // Total number of distance readings to log
const int DIST_READS_PER_CYCLE = 40;        // Number of readings to take per cycle
int distLog[NUM_DIST_READINGS - 1] = {};    // A log of previous distance sensor readings
int distLogIndex = 0;                       // Index of distanceLog

// Pins
const int LED_PIN = 13;               // Pin connected to LED
const int BUTTON_PIN = 8;             // Pin connected to red buttton
const int POT_PIN = A0;               // Pin connected to potmeter
const int DIST_SENSOR_PIN = A1;       // Pin connected to distance sensor
const int SERVO_PIN = 10;             // Pin connected to servo

// Control variables
int servoPos = 0;
int potPos = 0;
int error = 0;
int previousError = 0;
int setpoint = 0;
int integral = 0;
int ballDistance = 0;

/** Lookup table
    DISTANCE_TABLE = analog distance sensor values
    DISTANCE_INDEX = distance in centimetres
*/
int DISTANCE_TABLE[] = {620, 595, 569, 549, 529, 506, 482, 465, 447, 429, 411, 398, 384, 373, 362, 352, 341, 332, 322, 314, 306, 300, 293, 285, 277, 274, 270, 264, 257, 252, 246, 242, 238, 233, 228, 224, 219, 218, 216, 213, 210, 207, 204, 203, 202, 200, 197, 194, 190, 188, 186, 184, 181, 181, 180, 179, 177, 175, 172, 171, 169, 168, 166, 164, 162};
float DISTANCE_INDEX[] = {7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17, 17.5, 18, 18.5, 19, 19.5, 20, 20.5, 21, 21.5, 22, 22.5, 23, 23.5, 24, 24.5, 25, 25.5, 26, 26.5, 27, 27.5, 28, 28.5, 29, 29.5, 30, 30.5, 31, 31.5, 32, 32.5, 33, 33.5, 34, 34.5, 35, 35.5, 36, 36.5, 37, 37.5, 38, 38.5, 39};


void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  mainServo.attach(SERVO_PIN);    // attaches the servo on pin 9 to the servo object
  setServoPos(MID_SERVO_POS);     // enter balanced position
  Serial.begin(9600);
  for (int i = 0; i < 10; i++) {
    getDistance(DIST_SENSOR_PIN);          // populate the distance log for later use
  }
}


void loop() {
  potPos = analogRead(POT_PIN);
  ballDistance = constrain(getDistance(DIST_SENSOR_PIN), MAX_BALL_DISTANCE, MIN_BALL_DISTANCE);
  setLedPin(LED_PIN);

  switch (currentMode) {
    case MANUAL_MODE:
      manualMode();
      if (!isButtonPressed(BUTTON_PIN)) {
        currentMode = AUTO_MODE;
      }
      break;
    case AUTO_MODE:
      autoMode();
      if (!isButtonPressed(BUTTON_PIN)) {
        currentMode = MANUAL_MODE;
      }
      break;
    default:
      break;
  }
  printToSerial();
}

/** Control the servo directly with the potentiometer

*/
void manualMode() {
  setServoPos(map(analogRead(POT_PIN), 0, 1023, 0, 180));
}

/** Keeps the ball in position through  a control loop
    The beam angle is regulated so that the ball seeks out the setpoint,
    given by the potentiometer

*/
void autoMode() {
  setpoint = map(potPos, 0, 1023, MAX_BALL_DISTANCE, MIN_BALL_DISTANCE);
  error = setpoint - ballDistance;
  p = K_P * error;
  i = K_I * (i + error);
  d = K_D * (error - previousError);
  setServoPos(MID_SERVO_POS + p + i + d);
  previousError = error;
}

/** Set the position of the servo
    The servo has a range of 0-180, but is constrained due to physical limitations

    @param pos the position the servo will go to
*/
void setServoPos(int pos) {
  pos = constrain(pos, MIN_SERVO_POS, MAX_SERVO_POS);   // Make sure that pos is within the range 0 to 180
  servoPos = pos;
  mainServo.write(pos);
}

/** Sets the LED depending on the mode

   @param mode The current mode
   @param pin The pin of the LED to be used
*/
void setLedPin(int pin) {
  switch (currentMode) {
    case MANUAL_MODE:
      digitalWrite(pin, LOW);
      break;
    case AUTO_MODE:
      digitalWrite(pin, HIGH);
      break;
    default:
      break;
  }
}

/** Get the ball distance as an analog value

   @return the running average of the distance readings
*/
int getDistance(int sensorPin) {
  int distSum = 0;
  for (int i = 0; i < DIST_READS_PER_CYCLE; i++) {
  if (distLogIndex < NUM_DIST_READINGS) {
    distLog[distLogIndex] = analogRead(sensorPin);
    distLogIndex++;
  } else {
    distLogIndex = 0;
    distLog[distLogIndex] = ballDistance;
  }
  }
  for (int i = 0; i < NUM_DIST_READINGS; i++) {
    distSum = distSum + distLog[i];
  }
  ballDistance = (distSum / NUM_DIST_READINGS);
  return ballDistance;
}

/** Checks the lookup table to convert analog distance readings to centimetres
 *  
 *  @param distance The distance as an analog value between 0-1023
 *  @return the distance in centimetres
 */
float convertDistanceToCm (int distance) {
  float distanceInCm = 0.0;
  for (int i = 0; i < sizeof(DISTANCE_INDEX-1); i++) {
    if (distance <= DISTANCE_TABLE[i] && distance > DISTANCE_TABLE[i+1]) {
      distanceInCm = DISTANCE_TABLE[i];
    }
  }
  return distanceInCm;
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

/** Print relevant info to the serial monitor
   @param manualMode sets the mode between manual and automatic
*/
void printToSerial() {
  Serial.print("Distance (A): ");
  Serial.print(ballDistance);
  Serial.print(" (");
  Serial.print(convertDistanceToCm(ballDistance));
  Serial.print("cm)     ");
  Serial.print("Servo pos: ");
  Serial.print(servoPos);
  Serial.print("     ");

  switch (currentMode) {
    case (MANUAL_MODE):
      Serial.print("Pot val: ");
      Serial.print(potPos);
      break;
    case (AUTO_MODE):
      Serial.print("Setpoint: ");
      Serial.print(setpoint);
      Serial.print("     ");
      Serial.print("Error:");
      Serial.print(error);
      break;
    default:
      break;
  }
  Serial.println();
}
