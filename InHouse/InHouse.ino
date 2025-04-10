#include <PRIZM.h>
#include "PDController.h"
PRIZM p;

//I had fun
PDController elevatorPDController(0, 0.); //initialize PD controller object, tune

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
int ultraSonicSensorPort = 3;
int lineFinderSensorPort = 4;


void setup() {
  p.PrizmBegin(); //Intialize prizm
  
  // Reset encoders
  p.resetEncoder(leftEncoderPort);
  p.resetEncoder(elevatorEncoderPort);
  
  elevatorPDController.setTolerance(0); //tune
  
}

void loop() {
  totalRotations(); // Track h drive servo rotation
  p.setMotorPower(elevatorMotorPort, 100 * elevatorPDController.calculate(getDistance(elevatorEncoderPort)); //Calculates needed PD output
}

double getDistance(int channel) {
  double ticks = p.readEncoderCount(channel); //read encoder ticks
  return ticks / encoderTicksToInches[channel - 1]; //Returns the inches traveled, adjusts for array indexing
}

void moveElevatorToPosition(double targetPosition) {
  elevatorPDController.setSetpoint(targetPosition); //Sets elevator setpoint to new position
}

void waitUntilElevatorInPosition() {
  //While loop that just stalls code until elevator is ready
  while (!elevatorPDController.atSetpoint()) {}
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
  return cumulativeRotations / SERVO_ROTATIONS_PER_INCH; //Finds ditacne by dividing total rotations by conversion factor
}

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
