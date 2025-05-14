#include <PRIZM.h>
#include "PIDController.h"
#include "Constants.h" 
#include "math.h"

PRIZM p; //Initialize prizm object

//Creates state trackers
Constants::ClawState clawState = Constants::ClawState::CLOSE;

void setup() {
  //setup
  p.PrizmBegin(); //Start prizm
  Serial.begin(9600);

  Serial.println("start");
  
  //Reset encoders
  p.resetEncoder(Constants::driveEncoderPort);

  p.setMotorInvert(1, 1);

  //setup end

  //routine
  rack();
}

void loop() {}

void drive(int power) {
  if (power == 125) {
    p.setMotorPowers(power, power);
    return;
  }

  int leftSpeed = power / 1.8;
  int rightSpeed = power;
  p.setMotorPowers(leftSpeed, rightSpeed);
}

void waitUntilClawInPosition(double timeout) {
    unsigned long startTime = millis() / 1000; //Gets start time
  //While loop that just stalls code until elevator is ready
  while (!clawAtSetpoint()) {
    Serial.println(p.readServoPosition(Constants::clawServoPort));
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
  double startDistance = getDistance(true); //Finds the starting distance 
  double currentDistance = startDistance; //Initiaizes current distance variable
  
  //Cheks if we've traveled desired distance
  while (abs(currentDistance - startDistance) < inches) {
    p.setMotorPowers(power, power); //Sets motor powers
    currentDistance = getDistance(false); //Updates current distance
  }

  p.setMotorPower(Constants::rightDrivePort, 125); //brakes when done
}

double getDistance(bool reset) {
  if (reset) {
    p.resetEncoder(Constants::driveEncoderPort); //Resets
  }

  double ticks = p.readEncoderCount(Constants::driveEncoderPort); //read encoder ticks
  return ticks / Constants::driveEncoderTicksToInches; //Converts ticks to inches based off of conversion factor
}

//Sets claw to target state
void claw(Constants::ClawState target) {
  clawState = target;
  p.setServoPosition(Constants::clawServoPort, target);
}

void park() {
  drive(100);
  delay(2000);
  drive(125);
}

void supporting() {
  driveMotorDistance(30, 100);
  claw(Constants::ClawState::OPEN);
}

void supportingPark() {
  supporting();
  driveMotorDistance(30, -100);
  rotateCCW90();
  park();
}

void rack() {
  drive(100);
  delay(1000);
  drive(125);

  delay(250);
  rotateCW90();

  drive(100);
  delay(500);
  drive(125);

  delay(125);
  rotateCCW90();

  drive(100);
  delay(750);
  drive(125);

  delay(250);
  rotateCW90();

  p.setCRServoState(Constants::wristServoPort, 100);

  delay(250);
  drive(100);
  delay(1500);

  claw(Constants::OPEN);
  waitUntilClawInPosition(3);
  p.setCRServoState(Constants::wristServoPort, -100);
  delay(1000);
  drive(-100);
  delay(500);
  drive(125);
}

void rotateCCW90() {
  p.setMotorPowers(-100, 100);
  delay(550);
  p.setMotorPowers(125, 125);
}

void rotateCW90() {
  p.setMotorPowers(100, -100);
  delay(450);
  p.setMotorPowers(125, 125);
}

