#include <Servo.h>

Servo mainServo;      // Create servo object to control a servo

// Settings
const int MANUAL_MODE = 1;
const int AUTO_MODE = 2;
int currentMode = AUTO_MODE;

// Physical limitations
const int MIN_SERVO_POS = 90;          // Lowest allowed servo position
const int MAX_SERVO_POS = 155;         // Highest allowed servo position
const int MID_SERVO_POS = 115;         // Middle / stable servo position
const int MIN_BALL_DISTANCE = 500;     // (analog value) the closest the ball is allowed to be
const int MAX_BALL_DISTANCE = 190;     // (analog value) the farthest the ball is allowed to be

// Log of distance readings
const int NUM_DIST_READINGS = 20;           // Total number of distance readings to log
const int DIST_READS_PER_CYCLE = 20;        // Number of readings to take per cycle
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
int ballDistance = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  mainServo.attach(SERVO_PIN);    // attaches the servo on pin 9 to the servo object
  setServoPos(MID_SERVO_POS);     // enter balanced position
  Serial.begin(9600);
  for (int i = 0; i < 10; i++) {
    getDistance(DIST_SENSOR_PIN); // populate the distance log for later use
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
  
  setServoPos(MID_SERVO_POS);
}

/** Set the position of the servo
    The servo has a range of 0-180, but is constrained due to physical limitations

    @param pos the position the servo will go to
*/
void setServoPos(int pos) {
  pos = constrain(pos, MIN_SERVO_POS, MAX_SERVO_POS);   // Make sure that pos is within the range MIN_SERVO_POS to MAX_SERVO_POS
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
