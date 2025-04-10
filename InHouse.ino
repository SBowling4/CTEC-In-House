#include <PRIZM.h>
PRIZM p;

double encoderTicks[3] = {0, 0, 0};

double SERVO_ROTATIONS_TO_INCHES = 0;
double cumulativeRotations;

double previousRotation = 0;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  totalRotations();
}

double getDistance(int channel) {
  long ticks = p.readEncoderCount(channel); 
  return ticks / encoderTicks[channel];
}

void driveMotorDistance(int channel, long inches, int power) {
  long distance = getDistance(channel);
  while (distance < channel) {
    p.setMotorPower(channel, power);
    distance = getDistance(channel);
  }

  p.setMotorPower(channel, 125);
}

double getHDistance(long inches, int power) {
  p.readServoPosition(int channel)
}

void totalRotations() {
    double currentRotation = p.readServoPosition(3);

    // Calculate the delta, accounting for wraparound
    double delta = currentRotation - previousRotation;

    // Adjust for wraparound cases
    if (delta > 0.5) {
        delta -= 1.0; // Wrapped from 1.0 to 0.0 (moving backward)
    } else if (delta < -0.5) {
        delta += 1.0; // Wrapped from 0.0 to 1.0 (moving forward)
    }

    // Update cumulative position without relying on fullRotations counter
    cumulativeRotations -= delta; // Keep negative sign if needed for direction

    // Store for next calculation
    previousRotation = currentRotation;
}



