#include "Constants.h"

//Define port definitions
const int Constants::driveMotorsPort = 1;
const int Constants::elevatorMotorsPort = 2;
const int Constants::driveEncoderPort = 1;
const int Constants::elevatorEncoderPort = 2;
const int Constants::hServoPort = 1;
const int Constants::clawServoPort = 1;
const int Constants::wristServoPort = 2;
const int Constants::sonicSensorPort = 1;
const int Constants::lineFinderSensorPort = 2;

//Define servo conversion constants
const double Constants::hDriveServoRotationsPerInch = 0.0; //tune

//Define encoder constants
const double Constants::driveEncoderTicksToInches = 0.0; //tune
const double Constants::elevatorEncoderTicksToInches = 0.0; //tune

const double Constants::elevatorTolerance = 0.0;
const double Constants::wristTolerance = 0.0;
const double Constants::clawTolerance = 0.0;

const double Constants::elevatorkP = 0.0;
const double Constants::elevatorkI = 0.0;
const double Constants::elevatorkD = 0.0;

const double Constants::wristkP = 0.0;
const double Constants::wristkI = 0.0;
const double Constants::wristkD = 0.0;

