#include "PDController.h"
#include <Arduino.h>
#include <stdlib.h>

PDController::PDController(double kp, double kd) {
    this->kp = kp;
    this->kd = kd;
    this->lastTime = 0;
    this->lastError = 0;
    this->tolerance = 0;
}

PDController::PDController() {
    this->kp = 0;
    this->kd = 0;
    this->lastTime = 0;
    this->lastError = 0;
    this->tolerance = 0;
}

PDController::PDController(double kp) {
    this->kp = kp;
    this->kd = 0;
    this->lastTime = 0;
    this->lastError = 0;
    this->tolerance = 0;
}

double PDController::calculate(double measure) {
    // Get current time
    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    if (dt <= 0) dt = 0.001; // Avoid division by zero
    
    lastTime = currentTime;

    double error = setpoint - measure;
    double de = error - lastError;
    lastError = error;

    return kp * error + kd * (de/dt);
}

void PDController::setTolerance(double tolerance) {
    this->tolerance = tolerance;
}

bool PDController::atSetpoint() {
    if (tolerance != 0) {
        return abs(setpoint - lastError) < tolerance;
    } else {
        return false;
    }
}

void PDController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}
