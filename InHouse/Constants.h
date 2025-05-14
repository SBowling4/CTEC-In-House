#ifndef CONSTANTS_H
#define CONSTANTS_H

class Constants {
  public:
    //Port definitions
    static const int rightDrivePort;
    static const int leftDrivePort;
    static const int driveEncoderPort;
    static const int clawServoPort;
    static const int wristServoPort;


    //Servo conversion
    static const double driveEncoderTicksToInches;

    static const double clawTolerance;


    enum ClawState {
      OPEN = 125,
      CLOSE = 90,
    };


};

#endif 
