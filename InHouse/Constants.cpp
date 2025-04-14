#include "Constants.h"

//Define port definitions
const int Constants::driveMotorsPort = 1;
const int Constants::elevatorMotorsPort = 2;
const int Constants::driveEncoderPort = 1;
const int Constants::elevatorEncoderPort = 2;
const int Constants::hServoPort = 3;
const int Constants::clawServoPort = 4;
const int Constants::wristServoPort = 5;
const int Constants::ultraSonicSensorPort = 6;
const int Constants::lineFinderSensorPort = 7;

//Define servo conversion constants
const double Constants::hDriveServoRotationsPerInch = 0.0; //tune

//Define encoder constants
const double Constants::driveEncoderTicksToInches = 0.0; //tune
const double Constants::elevatorEncoderTicksToInches = 0.0; //tune

