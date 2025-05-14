#include "Constants.h"
#include "math.h"

//Define port definitions
const int Constants::rightDrivePort = 1;
const int Constants::leftDrivePort = 2;
const int Constants::driveEncoderPort = 1;
const int Constants::clawServoPort = 1;

const int Constants::wristServoPort = 2;

//Define encoder constants
const double Constants::driveEncoderTicksToInches = (2 * M_PI * 2.5) / 2048; //tune

const double Constants::clawTolerance = 5.0;


