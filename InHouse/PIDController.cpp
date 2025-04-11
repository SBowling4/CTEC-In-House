#include "PIDController.h"
#include <Arduino.h>
#include <stdlib.h>

PIDController::PIDController(double kp, double kd) {
    this->kp = kp;
    this->kd = kd;
    this->ki = 0;
    this->lastTime = 0;
    this->lastError = 0;
    this->tolerance = 0;
    this->integral = 0;
}

PIDController::PIDController() {
    this->kp = 0;
    this->kd = 0;
    this->ki = 0;
    this->lastTime = 0;
    this->lastError = 0;
    this->tolerance = 0;
    this->integral = 0;

}

PIDController::PIDController(double kp) {
    this->kp = kp;
    this->kd = 0;
    this->ki = 0;
    this->lastTime = 0;
    this->lastError = 0;
    this->tolerance = 0;
    this->integral = 0;

}

PIDController::PIDController(double kp, double ki, double kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->lastTime = 0;
  this->lastError = 0;
  this->tolerance = 0;
  this->integral = 0;

}

double PIDController::calculate(double measure) {
    // Get current time
    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    if (dt <= 0) dt = 0.001; // Avoid division by zero
    
    lastTime = currentTime;

    double error = setpoint - measure;
    double de = error - lastError;
    lastError = error;

    integral += error;

    return kp * error + kd * (de/dt) + integral * ki;
}

void PIDController::setTolerance(double tolerance) {
    this->tolerance = tolerance;
}

bool PIDController::atSetpoint() {
    if (tolerance != 0) {
        return abs(setpoint - lastError) < tolerance;
    } else {
        return false;
    }
}

void PIDController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}
