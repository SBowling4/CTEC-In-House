#include "PIDController.h"
#include <Arduino.h>
#include <stdlib.h>

PIDController::PIDController(double kp, double kd) {
  PIDController(kp, 0, kd);
}

PIDController::PIDController() {
  PIDController(0, 0, 0);
}

PIDController::PIDController(double kp) {
  PIDController(kp, 0, 0);
}

PIDController::PIDController(double kp, double ki, double kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

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
    
    lastTime = currentTime; //Updates kast time

    error = setpoint - measure; //Finds the error 
    double de = error - lastError; //Finds the difference in error
    lastError = error; //Updates last error to current error

    totalError = constrain((totalError + error) * dt, minimumIntegral / ki, maximumIntegral / ki); //Adds to total error, clamps it

    double derivative = de/dt; //Calculates the derivative of error

    return kp * error + kd * derivative + ki * totalError; //Calculates output
}

void PIDController::setTolerance(double tolerance) {
    this->tolerance = tolerance;
}

bool PIDController::atSetpoint() {
    if (tolerance != 0) {
        return abs(error) < tolerance;
    } else {
        return false;
    }
}

void PIDController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
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
