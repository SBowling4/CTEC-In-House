#ifndef CONSTANTS_H
#define CONSTANTS_H

class Constants {
  public:
    //Port definitions
    static const int driveMotorsPort;
    static const int elevatorMotorsPort;
    static const int driveEncoderPort;
    static const int elevatorEncoderPort;
    static const int hServoPort;
    static const int clawServoPort ;
    static const int wristServoPort;
    static const int ultraSonicSensorPort;
    static const int lineFinderSensorPort;

    //Servo conversion
    static const double hDriveServoRotationsPerInch;
    static const double driveEncoderTicksToInches;
    static const double elevatorEncoderTicksToInches;
     

    //Enum for wrist state
    enum WristState {
        UP = 0,
        DOWN = 0,
    };

    enum ClawState {
      OPEN = 0,
      CLOSE = 0,
    };

    enum ElevatorState{
      SUPPORTING = 0,
      L2 = 0,
      L3 = 0,
      L4 = 0,
      INTAKE = 0,
      BASE = 0,
    };


};

#endif 
