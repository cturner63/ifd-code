// Intermediate Fatigue Device Arduino Code
// Version 3 - Last Edited: Mar 13 23
// Originally written by Caroline Turner
// Collaborators: Joe Morton

// ==============================================================================================//

// ---------- INSTRUCTIONS ---------- //

// ==============================================================================================//

// Include vital libraries
#include <Stepper.h>  // stepper motor library
#include <SR04.h>     // ultrasonic sensor library

// ==============================================================================================//

// ---------- START READ-ONLY VALUES ---------- //

// Define all READ-ONLY linear actuator vars
const int fwdPin = 2;  // pin that correlates with moving the linear actuator forward
const int revPin = 3;  // pin that correlates with moving the linear actuator backward

// Define all READ-ONLY electromagnet vars
const int electroPinEngage = 6;     // pin that correlates with engaging the electromagnet
const int electroPinDisengage = 7;  // pin that correlates with disengaging the electromagnet

// Define all READ-ONLY ultrasonic sensor vars
const int trigPin = 12;
const int echoPin = 13;
SR04 sr04 = SR04(echoPin, trigPin);
long x;

// Define all READ-ONLY limit switch vars
const int limitSwitchPin = 5;
const int backLimitSwitchPin = 11;

// Define all READ-ONLY stepper motor vars
const int stepsPerRevolution = 1600;           // 200 steps/revolution for a NEMA 23
Stepper myStepper(stepsPerRevolution, 8, 9);  // pins that correlate to stepper motor

// Define all READ-ONLY button vars
const int buttonPin = 4;  // System button on/off
const int killPin = 10;   // Kill Button
bool systemState;         // Create system state


// ----------- END READ-ONLY VALUES ----------- //

// ==============================================================================================//

// ---------- START EDITABLE VALUES ---------- //

int startButtonState = 0;     // State of start button
int killButtonState = 0;      // State of kill button
int currentLSState;           // the current limit switch state
int currentBackLSState;       // the current back limit switch statew
int cycleCount = 0;           // initializing cycle count
int cyclesDesired = 100;        // (suggested val: X - Y )
int minDist = 7;              // distance from front datum of ultrasonic sensor to incident bar when gas spring is fully extended (cm)
int pullBackDist = 11;  // distance that you want the linear actuator to travel backwards for each shot (cm)


// ---------- END EDITABLE VALUES ---------- //

// ==============================================================================================//

void setup() {
  // Set the winder speed at a particular RPM:
  myStepper.setSpeed(100);

  // Open serial port at 9600 baud
  Serial.begin(9600);

  // Initialize the pushbutton pin as an input with an internal pullup resistor:
  pinMode(buttonPin, INPUT_PULLUP);           // set arduino pin to input pull-up mode
  startButtonState = digitalRead(buttonPin);  // read new state

  // Initialize the kill button pin as an input with an internal pullup resistor
  pinMode(killPin, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  killButtonState = digitalRead(killPin);

  // Initialize the limit switch pin as an input with an internal pullup resistor:
  pinMode(limitSwitchPin, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  currentLSState = digitalRead(limitSwitchPin);

  // Initialize the back limit switch pin as an input with an internal pullup resistor:
  pinMode(backLimitSwitchPin, INPUT_PULLUP);
  currentBackLSState = digitalRead(backLimitSwitchPin);


  // ---------- START OUTPUT (LOW-Z) PINS ---------- //

  // Declare electromagnet pins as OUTPUTS
  pinMode(electroPinEngage, OUTPUT);
  pinMode(electroPinDisengage, OUTPUT);

  // Declare linear actuator pins as OUTPUTS
  pinMode(fwdPin, OUTPUT);
  pinMode(revPin, OUTPUT);
  digitalWrite(fwdPin, LOW);
  digitalWrite(revPin, LOW);
  // ---------- END OUTPUT (LOW-Z) PINS ---------- //

  systemState = 0;
}

void loop() {

  // --------- TURN SYSTEM ON AND OFF DEPENDING ON BUTTON STATE --------- //

  // Read button states
  startButtonState = digitalRead(buttonPin);  // read new state
  killButtonState = digitalRead(killPin);
  currentBackLSState = digitalRead(backLimitSwitchPin);
  // Serial.print("Back LS Switch = ");
  // Serial.println(currentBackLSState);
  // delay(5000);


  // Toggle system state
  if (startButtonState == HIGH) {
    Serial.println("The start button is pressed");
    // Switch System State to HIGH
    systemState = 1;
  } else if (killButtonState == HIGH) {
    Serial.println("The kill button is pressed");
    systemState = 0;
  }

  // Switch Case Based on Button
  switch (systemState) {
    case 1:
      // check if below cycles desired
      // Measure distance of incident block
      // x = sr04.Distance();
      // Serial.print(x);
      // Serial.println("cm");

      // // Read LS
      // currentLSState = digitalRead(limitSwitchPin);
      // Serial.println(currentLSState);

      // Print system is running
      Serial.println("System is running");
      delay(2000);

      // Check if cycle count is reached
      if (cycleCount < cyclesDesired) {
        systemState = grabIncidentBlock(sr04, minDist, currentLSState, limitSwitchPin, systemState, killPin, killButtonState);
        systemState = pullIncidentBlock(sr04, pullBackDist, currentBackLSState, backLimitSwitchPin, systemState, killPin, killButtonState);
        systemState = windProjectile(myStepper, systemState, killPin, killButtonState);
        systemState = throwProjectile(electroPinEngage, electroPinDisengage, systemState, killPin, killButtonState);
        cycleCount++;  //+1 cycle completed
        Serial.print("cycles completed = ");
        Serial.println(cycleCount);
        Serial.print("cycles remaining = ");
        Serial.println(cyclesDesired-cycleCount);
        delay(1000);

      } else {
        systemState = 0;
        cycleCount = 0;
        return;
      }
      break;
    case 0:
      // End system like a kill switch
      Serial.println("System is off");
      // Stop LA
      digitalWrite(fwdPin, LOW);
      digitalWrite(revPin, LOW);

      // turn off EM
      digitalWrite(electroPinEngage, LOW);
      digitalWrite(electroPinDisengage, HIGH);

      break;
  }
}
// ==============================================================================================//

int grabIncidentBlock(SR04 sr04, int minDist, int currentLSState, int limitSwitchPin, int systemState, int killPin, int killButtonState) {
  while (true) {
    // Read kill switch state
    killButtonState = digitalRead(killPin);
    // Serial.print("kill button state = ");
    // Serial.println(killButtonState);

    if (killButtonState == 1) {
      systemState = 0;
    }

    // If kill switch pressed terminate function
    if (systemState == 0) {
      return systemState;
    }

    // Read x distance
    int x = sr04.Distance();
    // Serial.print("Current distance = ");
    // Serial.print(x);
    // Serial.println("cm");

    //extend linear actuator fully
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);

    // read the limit switch
    currentLSState = digitalRead(limitSwitchPin);
    // Serial.print("Current LS State = ");
    // Serial.println(currentLSState);

    // Check if at min distance
    if (x <= minDist && currentLSState == HIGH) {

      // stop linear actuator movement
      digitalWrite(fwdPin, LOW);
      digitalWrite(revPin, LOW);

      // turn on EM
      digitalWrite(electroPinEngage, HIGH);
      digitalWrite(electroPinDisengage, LOW);
      Serial.println("EM engaged");

      // end function
      Serial.println("Incident Block Grabbed");
      delay(2000);
      return systemState;

    } else {
      // Continue loop
      Serial.println("Grabbing Incident Block");
      continue;
    }
  }
}

int pullIncidentBlock(SR04 sr04, int pullBackDist, int currentBackLSState, int backLimitSwitchPin, int systemState, int killPin, int killButtonState) {
  while (true) {
    // Read switch states
    killButtonState = digitalRead(killPin);
    currentBackLSState = digitalRead(backLimitSwitchPin);
    // Serial.print("Back LS Switch = ");
    // Serial.println(currentBackLSState);
    // delay(5000);

    // Conditions
    if (killButtonState == 1) {
      systemState = 0;
    } else if (currentBackLSState == 1) {
      systemState = 0;
      Serial.println("Incident Block not Pulled back or Limit reached");
    }

    // If kill switch or lim switch pressed terminate function
    if (systemState == 0) {
      return systemState;
    }

    // Read x distance
    int x = sr04.Distance();
    Serial.print("Current distance = ");
    Serial.print(x);
    Serial.println("cm");

    // go to set distance
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);

    if (x >= pullBackDist) {

      // stop linear actuator
      digitalWrite(fwdPin, LOW);
      digitalWrite(revPin, LOW);

      // End Function
      Serial.println("Distance Reached");
      return systemState;

    } else {
      // Continue loop
      Serial.println("Pulling Back Block");
      continue;
    }
  }
}

// ==============================================================================================//

int windProjectile(Stepper myStepper, int systemState, int killPin, int killButtonState) {
  // Read kill switch state
  killButtonState = digitalRead(killPin);
  if (killButtonState == 1) {
    systemState = 0;
  }

  // If kill switch pressed terminate function
  if (systemState == 0) {
    return systemState;
  }

  // winder winds projectile back
  Serial.println("Winding Projectile");
  myStepper.step(4*stepsPerRevolution);

  // Time delay before other steps
  delay(1000);

  return systemState;
}

// ==============================================================================================//

int throwProjectile(int electroPinEngage, int electroPinDisengage, int systemState, int killPin, int killButtonState) {
  // Read kill switch state
  killButtonState = digitalRead(killPin);
  if (killButtonState == 1) {
    systemState = 0;
  }

  // If kill switch pressed terminate function
  if (systemState == 0) {
    return systemState;
  }

  // turn off EM
  Serial.println("EM Turned off");
  digitalWrite(electroPinEngage, LOW);
  digitalWrite(electroPinDisengage, HIGH);
  return systemState;
}
