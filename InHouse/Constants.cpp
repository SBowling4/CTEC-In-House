#include "Constants.h"
#include "math.h"

//Define port definitions
const int Constants::rightDrivePort = 1;
const int Constants::leftDrivePort = 2;
const int Constants::driveEncoderPort = 1;
const int Constants::hServoPort = 2;
const int Constants::clawServoPort = 1;
const int Constants::wristServoPort = 1;

//Define servo conversion constants
const double Constants::hDriveServoRotationsPerInch = 0.0; //tune

//Define encoder constants
const double Constants::driveEncoderTicksToInches = (2 * M_PI * 2.5) / 2048; //tune

const double Constants::wristTolerance = 0.0;
const double Constants::clawTolerance = 5.0;

const double Constants::wristkP = 0.0;
const double Constants::wristkI = 0.0;
const double Constants::wristkD = 0.0;

