#ifndef CONSTANTS_H
#define CONSTANTS_H

class Constants {
  public:
    //Port definitions
    static const int rightDrivePort;
    static const int leftDrivePort;
    static const int driveEncoderPort;
    static const int hServoPort;
    static const int clawServoPort;
    static const int wristServoPort;

    //Servo conversion
    static const double hDriveServoRotationsPerInch;
    static const double driveEncoderTicksToInches;

    static const double wristTolerance; 
    static const double clawTolerance;

    static const double wristkP;
    static const double wristkI;
    static const double wristkD;


     

    //Enum for wrist state
    enum WristState {
        UP = 0,
        DOWN = 0,
    };

    enum ClawState {
      OPEN = 125,
      CLOSE = 90,
    };


};

#endif 
