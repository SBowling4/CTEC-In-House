#include "PIDController.h"
#include <Arduino.h>
#include <stdlib.h>

//Constructor for kp and kd
PIDController::PIDController(double kp, double kd) {
  this->kP = kp;
  this->kI = 0;
  this->kD = kd;

  this->lastTime = 0;
  this->lastError = 0;
  this->totalError = 0;
  this->error = 0;
  this->tolerance = 0;
  this->maximumIntegral = 1;
  this->minimumIntegral = -1;
}

//Default constructor
PIDController::PIDController() {
  this->kP = 0;
  this->kI = 0;
  this->kD = 0;

  this->lastTime = 0;
  this->lastError = 0;
  this->totalError = 0;
  this->error = 0;
  this->tolerance = 0;
  this->maximumIntegral = 1;
  this->minimumIntegral = -1;
}

//Constructor with just kp
PIDController::PIDController(double kp) {
  this->kP = kp;
  this->kI = 0;
  this->kD = 0;

  this->lastTime = 0;
  this->lastError = 0;
  this->totalError = 0;
  this->error = 0;
  this->tolerance = 0;
  this->maximumIntegral = 1;
  this->minimumIntegral = -1;
}

// Constructor with all parameters
PIDController::PIDController(double kp, double ki, double kd) {
  this->kP = kp;
  this->kI = ki;
  this->kD = kd;

  this->lastTime = 0;
  this->lastError = 0;
  this->totalError = 0;
  this->error = 0;
  this->tolerance = 0;
  this->maximumIntegral = 1;
  this->minimumIntegral = -1;
}

double PIDController::calculate(double measure) {
    unsigned long currentTime = millis(); //Get current time
    double dt = (currentTime - lastTime) / 1000.0; //Convert to seconds
    if (dt <= 0) dt = 0.001; //Avoid division by zero
    
    lastTime = currentTime; //Updates last time

    error = setpoint - measure; //Finds the error 
    double de = error - lastError; //Finds the difference in error
    lastError = error; //Updates last error to current error

    totalError = constrain(totalError + error * dt, minimumIntegral / kI, maximumIntegral / kI); //Adds to total error, clamps it

    double derivative = de/dt; //Calculates the derivative of error

    return kP * error + kD * derivative + kI * totalError; //Calculates output
}

void PIDController::setTolerance(double tolerance) {
  this->tolerance = tolerance; //Sets the tolerance
}

bool PIDController::atSetpoint() {
  if (tolerance != 0) { //Double checks tolerance has been set
      return abs(error) < tolerance;
  } else {
      return false;
  }
}

void PIDController::setSetpoint(double setpoint) {
  this->setpoint = setpoint; //Updates the setpoint
}

void PIDController::setIntegralRange(double min, double max) {
  this->minimumIntegral = min;
  this->maximumIntegral = max;
}

void PIDController::reset() {
  this->totalError = 0;
  this->lastError = 0;
  this->error = 0;
}
