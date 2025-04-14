#include <PRIZM.h>
#include "PIDController.h"
#include "Constants.h" 
#include <math.h>

PRIZM p;

//I made this myself, I can add the files to submission if you want
PIDController elevatorPIDController(0, 0, 0); //initialize PID controller object, tune

double cumulativeRotations = 0;
double previousRotation = 0;

Constants::WristState wristState = Constants::WristState::UP; 
Constants::ClawState clawState = Constants::ClawState::CLOSE;

void setup() {
  //setup
  p.PrizmBegin(); //Intialize prizm
  
  // Reset encoders
  p.resetEncoder(Constants::driveEncoderPort);
  p.resetEncoder(Constants::elevatorEncoderPort);

  elevatorPIDController.setTolerance(0.1); //tune, tolerance for elevator at position

  //setup end

  //routine
  supporting1();
}

void loop() {
  totalRotations(); // Track h-drive servo rotation
  p.setMotorPower(Constants::elevatorMotorsPort, 100 * elevatorPIDController.calculate(getDistance(Constants::elevatorEncoderPort, false))); //Calculates needed PID output
}

void moveElevatorToPosition(double targetPosition) {
  elevatorPIDController.setSetpoint(targetPosition); //Sets elevator setpoint to new position
}

void waitUntilElevatorInPosition(double timeout) {
  unsigned long startTime = millis() / 1000;
  //While loop that just stalls code until elevator is ready
  while (!elevatorPIDController.atSetpoint()) {
    unsigned long currentTime = millis() / 1000;
    if (currentTime - startTime > timeout) {
      Serial.println("Unable to move elevator to position within time, cancelling");
      break;
    }
  }
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
  
  p.setCRServoState(Constants::hServoPort, 0); // Stop servo
}

void driveH(int power) {
  p.setCRServoState(Constants::hServoPort, power);
}

void driveMotor(int power) {
  p.setMotorPower(Constants::driveMotorsPort, power);
}

double getDistance(int port, bool reset) {
  if (reset) {
    p.resetEncoder(port);
  }

  double ticks = p.readEncoderCount(port); //read encoder ticks
  return ticks / (port == 1 ? Constants::driveEncoderTicksToInches : Constants::elevatorEncoderTicksToInches);
}

double getHDistance(bool reset) {
  if (reset) {
    cumulativeRotations = 0;
    return cumulativeRotations / Constants::hDriveServoRotationsPerInch;
  }
  return cumulativeRotations / Constants::hDriveServoRotationsPerInch; //Finds distance by dividing total rotations by conversion factor
}

bool getLF() {
  return p.readLineSensor(Constants::lineFinderSensorPort); //Easier getter for LF sensor
}

int getUS() {
  return p.readSonicSensorCM(Constants::ultraSonicSensorPort);
}

void startToArray() {
  driveMotorDistance(40, 100);
}

void supporting() {
  elevatorPIDController.setSetpoint(Constants::ElevatorState::SUPPORTING);
  waitUntilElevatorInPosition(4);
  wrist(Constants::WristState::DOWN);
  claw(Constants::ClawState::OPEN);
}

void base() {
  elevatorPIDController.setSetpoint(Constants::ElevatorState::BASE);
  wrist(Constants::WristState::UP);
  claw(Constants::ClawState::OPEN);
}

void wrist(Constants::WristState target) {
  p.setServoPosition(Constants::wristServoPort, target);

  wristState = target;
}

void claw(Constants::ClawState target) {
  p.setServoPosition(Constants::clawServoPort, target);

  clawState = target;
}

void supporting1() {
  startToArray();
  supporting();
  base();
}

//look familiar mcleod?
void totalRotations() {
    double currentRotation = p.readServoPosition(Constants::hServoPort); //Gets the current rotation from servo

    // Calculate the delta, accounting for wraparound
    double delta = currentRotation - previousRotation;

    // Adjust for wraparound cases
    if (delta > 0.5) {
        delta -= 1.0; // Wrapped from 1.0 to 0.0 (moving backward)
    } else if (delta < -0.5) {
        delta += 1.0; // Wrapped from 0.0 to 1.0 (moving forward)
    }

    // Update cumulative position 
    cumulativeRotations += delta; 

    // Store for next
    previousRotation = currentRotation;
}
