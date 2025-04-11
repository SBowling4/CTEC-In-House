#include <PRIZM.h>
#include "PIDController.h"
PRIZM p;

//I made this myself, I can add the files to submission if you want
PIDController elevatorPIDController(0, 0, 0); //initialize PID controller object, tune

double encoderTicksToInches[2] = {0, 0}; //initialie conversion array, tune

//Servo calibrations
double SERVO_ROTATIONS_PER_INCH = 0; //tune 
double cumulativeRotations = 0;
double previousRotation = 0;

// Port definitions
int leftMotorPort = 1;
int elevatorMotorPort = 2;
int leftEncoderPort = 1;
int elevatorEncoderPort = 2;
int hServoPort = 3;
int clawServopPort = 4;
int wristServoPort = 5;
int ultraSonicSensorPort = 6;
int lineFinderSensorPort = 7;


void setup() {
  p.PrizmBegin(); //Intialize prizm
  
  // Reset encoders
  p.resetEncoder(leftEncoderPort);
  p.resetEncoder(elevatorEncoderPort);

  elevatorPIDController.setTolerance(0); //tune, tolerance for elevator at position
}

void loop() {
  totalRotations(); // Track h drive servo rotation
  p.setMotorPower(elevatorMotorPort, 100 * elevatorPIDController.calculate(getDistance(elevatorEncoderPort))); //Calculates needed PID output
}

double getDistance(int channel) {
  double ticks = p.readEncoderCount(channel); //read encoder ticks
  return ticks / encoderTicksToInches[channel - 1]; //Returns the inches traveled, adjusts for array indexing
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
  double startDistance = getDistance(leftEncoderPort); //Finds the starting distance 
  double currentDistance = startDistance; //Initiaizes current distance variable
  
  //Cheks if we've traveled desired distance
  while ((currentDistance - startDistance) < inches) {
    p.setMotorPower(leftMotorPort, power); //Sets motor powers
    currentDistance = getDistance(leftEncoderPort); //Updates current distance
  }

  p.setMotorPower(leftMotorPort, 125); //brakes when done
}

void driveHDistance(double inches, int power) {
  double startDistance = getHDistance(); //Finds starting distance
  double currentDistance = startDistance; //Initializes current distance
  
  while ((currentDistance - startDistance) < inches) {
    p.setCRServoState(hServoPort, power); //Sets h-drive servo to power
    currentDistance = getHDistance(); //Updates current distance
  }
  
  p.setCRServoState(hServoPort, 0); // Stop servo
}

double getHDistance() {
  return cumulativeRotations / SERVO_ROTATIONS_PER_INCH; //Finds distance by dividing total rotations by conversion factor
}

bool getLF() {
  return p.readLineSensor(lineFinderSensorPort); //Easier getter for LF sensor
}

void startToRack() {}

//look familiar mcleod?
void totalRotations() {
    double currentRotation = p.readServoPosition(hServoPort); //Gets the current rotation from servo

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
