#include <PRIZM.h>
#include "PIDController.h"
#include "Constants.h" 
#include <math.h>

PRIZM p;

//I made this myself, I can add the files to submission if you want
PIDController elevatorPIDController(0, 0, 0); //initialize PID controller object, tune

double cumulativeRotations = 0;
double previousRotation = 0;

Constants::WristState wristState = Constants::UP; 
Constant::ClawState clawState = Constants::CLOSE;

void setup() {
  p.PrizmBegin(); //Intialize prizm
  
  // Reset encoders
  p.resetEncoder(Constants::leftEncoderPort);
  p.resetEncoder(Constants::elevatorEncoderPort);

  elevatorPIDController.setTolerance(0); //tune, tolerance for elevator at position

  //setup end
  startToRack();
}

void loop() {
  totalRotations(); // Track h-drive servo rotation
  p.setMotorPower(Constants::elevatorMotorPort, 100 * elevatorPIDController.calculate(getDistance(Constants::elevatorEncoderPort))); //Calculates needed PID output
}

double getDistance(int channel) {
  double ticks = p.readEncoderCount(channel); //read encoder ticks
  return ticks / Constants::encoderTicksToInches[channel - 1]; //Returns the inches traveled, adjusts for array indexing
}

void moveElevatorToPosition(double targetPosition) {
  elevatorPIDController.setSetpoint(targetPosition); //Sets elevator setpoint to new position
}

void waitUntilElevatorInPosition() {
  unsigned long startTime = millis() / 1000;
  //While loop that just stalls code until elevator is ready
  while (!elevatorPIDController.atSetpoint()) {
    unsigned long currentTime = millis() / 1000;
    if (currentTime - startTime > 4) {
      Serial.println("Unable to move elevator to position within time, cancelling");
      break;
    }
  }
}

void driveLefthMotorDistance(double inches, int power) {
  double startDistance = getDistance(Constants::leftEncoderPort); //Finds the starting distance 
  double currentDistance = startDistance; //Initiaizes current distance variable
  
  //Cheks if we've traveled desired distance
  while (abs(currentDistance - startDistance) < inches) {
    p.setMotorPower(Constants::leftMotorPort, power); //Sets motor powers
    currentDistance = getDistance(Constants::leftEncoderPort); //Updates current distance
  }

  p.setMotorPower(Constants::leftMotorPort, 125); //brakes when done
}

void rotateRight90() {
  driveLeftMotorDistance(2 * M_PI * Constants::BOT_RADIUS_INCHES * (90/360), 100);
}

void rotateLeft90() {
  driveLeftMotorDistance(2 * M_PI * Constants::BOT_RADIUS_INCHES * (90/360), -100);
}

void driveHDistance(double inches, int power) {
  double startDistance = getHDistance(); //Finds starting distance
  double currentDistance = startDistance; //Initializes current distance
  
  while ((currentDistance - startDistance) < inches) {
    p.setCRServoState(Constants::hServoPort, power); //Sets h-drive servo to power
    currentDistance = getHDistance(); //Updates current distance
  }
  
  p.setCRServoState(Constants::hServoPort, 0); // Stop servo
}

void driveH(int power) {
  p.setCRServoState(Constants::hServoPort, power);
}

double getHDistance() {
  return cumulativeRotations / Constants::SERVO_ROTATIONS_PER_INCH; //Finds distance by dividing total rotations by conversion factor
}

bool getLF() {
  return p.readLineSensor(Constants::lineFinderSensorPort); //Easier getter for LF sensor
}

int getUS() {
  return p.readSonicSensorCM(Constants::ultraSonicSensorPort);
}

void startToRack() {
  while (!getLF()) {
    driveH(100);
  }
  driveHDistance(12, 100);
  rotateRight90();
  while (!getLF()) {
    driveHDistance(38, 100);
  }
}

void wrist(Constants::WristState target) {
  switch (target) {
    case Constants::UP:
      p.setServoPosition(Constants::wristServoPort, Constants::wristUpPos);
      break;
    case Constants::DOWN:
      p.setServoPosition(Constants::wristServoPort, Constants::wristDownPos);
      break;
  }

  wristState = target;
}

void claw(Constants::ClawState target) {
  switch (target) {
    case Constants::ClOSE:
      p.setServoPosition(Constants::clawServoPort, Constants::clawClosePos);
      break;
    case Constants::OPEN:
      p.setServoPosition(Constants::clawServoPort, Constants::clawOpenPos);
      break;
  }

  clawState = target;
}

void rackAuto() {
  startToRack();
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
