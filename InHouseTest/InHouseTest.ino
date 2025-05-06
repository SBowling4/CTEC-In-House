#include <PRIZM.h>
#include "PIDController.h"
#include "Constants.h" 

PRIZM p; //Initialize prizm object

//I made this myself, I can add the files to submission if you want
PIDController elevatorPIDController(Constants::elevatorkP, Constants::elevatorkI, Constants::elevatorkD); //initialize PID controller object, tune
PIDController wristPIDController(Constants::wristkP, Constants::wristkI, Constants::wristkD);

//Initializes h-drive rotation tracking
double cumulativeRotations = 0;
double previousRotation = 0;

//Creates state trackers
Constants::WristState wristState = Constants::WristState::UP; 
Constants::ClawState clawState = Constants::ClawState::CLOSE;
Constants::ElevatorState elevatorState = Constants::ElevatorState::BASE;

void setup() {
  //setup
  p.PrizmBegin(); //Start prizm
  Serial.begin(9600);
  
  //Reset encoders
  p.resetEncoder(Constants::driveEncoderPort);
  p.resetEncoder(Constants::elevatorEncoderPort);

  elevatorPIDController.setTolerance(Constants::elevatorTolerance); //tune, tolerance for elevator at position
  wristPIDController.setTolerance(Constants::wristTolerance);

  Serial.println("Setup end");

  //setup end

  //routine
  // supporting1DR();



}

void loop() {
  // totalRotations(); //Track h-drive servo rotation
  // p.setMotorPower(Constants::elevatorMotorsPort, 100 * elevatorPIDController.calculate(getDistance(Constants::elevatorEncoderPort, false))); //Calculates needed PID output
  // p.setCRServoState(Constants::wristServoPort, 100 * wristPIDController.calculate(p.readServoPosition(Constants::wristServoPort)));

  Serial.println("Start");
  p.setGreenLED(HIGH);
  p.setCRServoState(1, 25);
  delay(1000);
  p.setCRServoState(1, 0);
  p.setGreenLED(LOW);
  delay(500);
  Serial.println("End");
}

void waitUntilElevatorInPosition(double timeout) {
  unsigned long startTime = millis() / 1000; //Gets start time
  //While loop that just stalls code until elevator is ready
  while (!elevatorPIDController.atSetpoint()) {
    unsigned long currentTime = millis() / 1000; //Gets current time
    //If too much time has passed, leave
    if (currentTime - startTime > timeout) {
      Serial.println("Unable to move elevator to position within time, cancelling");
      break;
    }
  }
}

void waitUntilWristInPosition(double timeout) {
  unsigned long startTime = millis() / 1000; //Gets start time
  //While loop that just stalls code until elevator is ready
  while (!wristPIDController.atSetpoint()) {
    unsigned long currentTime = millis() / 1000; //Gets current time
    //If too much time has passed, leave
    if (currentTime - startTime > timeout) {
      Serial.println("Unable to move wrist to position within time, moving on");
      break;
    }
  }
}

void waitUntilClawInPosition(double timeout) {
    unsigned long startTime = millis() / 1000; //Gets start time
  //While loop that just stalls code until elevator is ready
  while (!clawAtSetpoint()) {
    unsigned long currentTime = millis() / 1000; //Gets current time
    //If too much time has passed, leave
    if (currentTime - startTime > timeout) {
      Serial.println("Unable to move claw to position within time, moving on");
      break;
    }
  }
}

bool clawAtSetpoint() {
  return abs(p.readServoPosition(Constants::clawServoPort) - clawState) <= Constants::clawTolerance;
}

void driveMotorDistance(double inches, int power) {
  double startDistance = getDistance(Constants::driveEncoderPort, true); //Finds the starting distance 
  double currentDistance = startDistance; //Initiaizes current distance variable
  
  //Cheks if we've traveled desired distance
  while (abs(currentDistance - startDistance) < inches) {
    p.setMotorPower(Constants::driveMotorsPort, power); //Sets motor powers
    currentDistance = getDistance(Constants::driveEncoderPort, false); //Updates current distance
  }

  p.setMotorPower(Constants::driveMotorsPort, 125); //brakes when done
}

void driveHDistance(double inches, int power) {
  double startDistance = getHDistance(true); //Finds starting distance
  double currentDistance = startDistance; //Initializes current distance
  
  while ((currentDistance - startDistance) < inches) {
    p.setCRServoState(Constants::hServoPort, power); //Sets h-drive servo to power
    currentDistance = getHDistance(false); //Updates current distance
  }
  
  p.setCRServoState(Constants::hServoPort, 0); //Stop servo
}

double getDistance(int port, bool reset) {
  if (reset) {
    p.resetEncoder(port); //Resets
  }

  double ticks = p.readEncoderCount(port); //read encoder ticks
  return ticks / (port == 1 ? Constants::driveEncoderTicksToInches : Constants::elevatorEncoderTicksToInches); //Converts ticks to inches based off of conversion factor
}

//Gets the distance the H servo has driven, resets when being initially called because it is always tracking its distance
double getHDistance(bool reset) {
  if (reset) {
    cumulativeRotations = 0; //Resets distance
    return cumulativeRotations / Constants::hDriveServoRotationsPerInch; //Finds distance by dividing total rotations by conversion factor
  }
  return cumulativeRotations / Constants::hDriveServoRotationsPerInch; //Finds distance by dividing total rotations by conversion factor
}

//Easier getter for LF sensor
int getLF() {
  return p.readLineSensor(Constants::lineFinderSensorPort); 
}

//Easier getter for sonic sensor
int getSonic() {
  return p.readSonicSensorCM(Constants::sonicSensorPort); 
}

//Drives from start to the array
void startToArray() {
  driveMotorDistance(40, 100); //Drives 40 inches, then stops
}

void lineToArray() {
  driveMotorDistance(30, 100);
}

void startToGrid() {
  driveHDistance(32, -100);
  while (getLF() == 0) {
    p.setMotorPower(Constants::driveMotorsPort, 100);
  }
}

void gridToPickup() {
  driveHDistance(1, -100);
  while (getLF() == 0) {
    p.setCRServoState(Constants::hServoPort, -100);
  }
  p.setCRServoState(Constants::hServoPort, 0);
}

void pickupToGrid() {
  driveHDistance(1, 100);
  while (getLF() == 0) {
    p.setCRServoState(Constants::hServoPort, 100);
  }
  p.setCRServoState(Constants::hServoPort, 0);
}

void arrayToLine() {
  while (getLF() == 0) {
    p.setMotorPower(Constants::driveMotorsPort, -100);
  }
  p.setMotorPower(Constants::sonicSensorPort, 125);
}

void closeLineToPark() {
  driveHDistance(34, -100);
}

//Moves superstructture to supporting beacon position, then scores
void supporting() {
  elevator(Constants::ElevatorState::SUPPORTING);
  wrist(Constants::WristState::DOWN);
  waitUntilElevatorInPosition(5);
  waitUntilWristInPosition(5);
  claw(Constants::ClawState::OPEN);
}

void l4() {
  elevator(Constants::ElevatorState::L4);
  wrist(Constants::WristState::UP);
  waitUntilElevatorInPosition(5);
  waitUntilWristInPosition(5);
  claw(Constants::ClawState::OPEN);
}

void l3() {
  elevator(Constants::ElevatorState::L4);
  wrist(Constants::WristState::UP);
  waitUntilElevatorInPosition(5);
  waitUntilWristInPosition(5);
  claw(Constants::ClawState::OPEN);
}

void l2() {
  elevator(Constants::ElevatorState::L4);
  wrist(Constants::WristState::UP);
  waitUntilElevatorInPosition(5);
  waitUntilWristInPosition(5);
  claw(Constants::ClawState::OPEN);
}

void intake() {
  claw(Constants::ClawState::OPEN);
  elevator(Constants::ElevatorState::INTAKE);
  wrist(Constants::WristState::DOWN);
  waitUntilElevatorInPosition(5);
  waitUntilWristInPosition(5);
  waitUntilClawInPosition(5);
  claw(Constants::ClawState::CLOSE);
}

//Moves superstructure to base position
void base() {
  elevator(Constants::ElevatorState::BASE);
  wrist(Constants::WristState::UP);
}

//Sets wrist to target state
void wrist(Constants::WristState target) {
  wristState = target;
  p.setServoPosition(Constants::wristServoPort, target);
}

//Sets claw to target state
void claw(Constants::ClawState target) {
  clawState = target;
  wristPIDController.setSetpoint(target);
}

void elevator(Constants::ElevatorState target) {
  elevatorState = target;
  elevatorPIDController.setSetpoint(target);
}

//1 energy cell supporting beacon auto, dead reckoning
void supporting1DR() {
  startToArray();
  supporting();
  base();
}

//I energy cell supporing beacon auto
void supporting1() {
  while (getLF() == 0) {
    p.setMotorPower(Constants::driveMotorsPort, 100);
  }
  lineToArray();
  supporting();
  base();
}

void supporting1Park() {
  while (getLF() == 0) {
    p.setMotorPower(Constants::driveMotorsPort, 100);
  }
  lineToArray();
  supporting();
  base();
  arrayToLine();
  closeLineToPark();
}

void gridL4(int pieces) {
  startToGrid();
  l4();
  base();
  for (int i = 0; i < pieces - 1; i++) {
    gridToPickup();
    intake();
    base();
    pickupToGrid();
    l4();
    base();
  }
}

void gridL4L3() {
  startToGrid();
  l4();
  base();
  gridToPickup();
  intake();
  base();
  pickupToGrid();
  l3();
}

void gridL4L3L2() {
  startToGrid();
  l4();
  base();
  gridToPickup();
  intake();
  base();
  pickupToGrid();
  l3();
  base();
  gridToPickup();
  intake();
  base();
  pickupToGrid();
  l2();

}
//look familiar mcleod?
void totalRotations() {
    double currentRotation = p.readServoPosition(Constants::hServoPort); //Gets the current rotation from servo

    //Calculate the delta, accounting for wraparound
    double delta = currentRotation - previousRotation;

    //Adjust for wraparound cases
    if (delta > 0.5) {
        delta -= 1.0; //Wrapped from 1.0 to 0.0 (moving backward)
    } else if (delta < -0.5) {
        delta += 1.0; //Wrapped from 0.0 to 1.0 (moving forward)
    }

    //Update cumulative position 
    cumulativeRotations += delta; 

    //Store for next
    previousRotation = currentRotation;
}
