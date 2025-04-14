#ifndef CONSTANTS_H
#define CONSTANTS_H

class Constants {
  public:
    //Port definitions
    static const int leftMotorPort = 1;
    static const int elevatorMotorPort = 2;
    static const int leftEncoderPort = 1;
    static const int elevatorEncoderPort = 2;
    static const int hServoPort = 3;
    static const int clawServoPort = 4; 
    static const int wristServoPort = 5;
    static const int ultraSonicSensorPort = 6;
    static const int lineFinderSensorPort = 7;

    //Servo conversion
    static const double SERVO_ROTATIONS_PER_INCH = 0.0; //tune 

    static const double encoderTicksToInches[2] = {0.0, 0.0}; 

    static const double BOT_RADIUS_INCHES = 0;

    //Enum for wrist state
    enum WristState {
        UP,
        DOWN
    };

    enum ClawState {
      OPEN,
      CLOSE
    };

    static const int wristUpPos = 0;
    static const int wristDownPos = 0;
    static const int clawOpenPos = 0;
    static const int clawClosePos = 0;

};

#endif 
