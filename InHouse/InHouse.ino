#include <PRIZM.h>
#include "PIDController.h"
#include "Constants.h" 
#include <math.h>

PRIZM p; //Initialize prizm object

//I made this myself, I can add the files to submission if you want
PIDController elevatorPIDController(0, 0, 0); //initialize PID controller object, tune

//Initializes h-drive rotation tracking
double cumulativeRotations = 0;
double previousRotation = 0;

//Creates state trackers
Constants::WristState wristState = Constants::WristState::UP; 
Constants::ClawState clawState = Constants::ClawState::CLOSE;
Constants::ElevatorState elevatorState = Constants::ElevatorState::BASE;

void setup() {
  //setup
  p.PrizmBegin(); //Intialize prizm
  
  //Reset encoders
  p.resetEncoder(Constants::driveEncoderPort);
  p.resetEncoder(Constants::elevatorEncoderPort);

  elevatorPIDController.setTolerance(Constants::elevatorTolerance); //tune, tolerance for elevator at position

  //setup end

  //routine
  supporting1();
}

void loop() {
  totalRotations(); //Track h-drive servo rotation
  p.setMotorPower(Constants::elevatorMotorsPort, 100 * elevatorPIDController.calculate(getDistance(Constants::elevatorEncoderPort, false))); //Calculates needed PID output
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
  while (!wristAtSetpoint()) {
    unsigned long currentTime = millis() / 1000; //Gets current time
    //If too much time has passed, leave
    if (currentTime - startTime > timeout) {
      Serial.println("Unable to move wrist to position within time, moving on");
      break;
    }
  }
}

bool wristAtSetpoint() {
  return abs(p.readServoPosition(Constants::wristServoPort) - wristState) < Constants::wristTolerance; //Returns if wrist is within tolerance 
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
int getUS() {
  return p.readSonicSensorCM(Constants::ultraSonicSensorPort); 
}

//Drives from start to the array
void startToArray() {
  driveMotorDistance(40, 100); //Drives 40 inches, then stops
}

//Moves superstructture to supporting beacon position, then scores
void supporting() {
  elevator(Constants::ElevatorState::SUPPORTING);
  wrist(Constants::WristState::DOWN);
  waitUntilElevatorInPosition(5);
  waitUntilWristInPosition(5);
  claw(Constants::ClawState::OPEN);
}

//Moves superstructure to base position
void base() {
  elevator(Constants::ElevatorState::BASE);
  wrist(Constants::WristState::UP);
  claw(Constants::ClawState::OPEN);
}

//Sets wrist to target state
void wrist(Constants::WristState target) {
  p.setServoPosition(Constants::wristServoPort, target);

  wristState = target;
}

//Sets claw to target state
void claw(Constants::ClawState target) {
  p.setServoPosition(Constants::clawServoPort, target);

  clawState = target;
}

void elevator(Constants::ElevatorState target) {
  elevatorPIDController.setSetpoint(target);
  elevatorState = target;
}

//1 energy cell supporting beacon auto
void supporting1() {
  startToArray();
  supporting();
  base();
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
