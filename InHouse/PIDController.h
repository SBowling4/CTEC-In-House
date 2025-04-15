#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
    public:
        double kP;
        double kD;
        double kI;

        double setpoint;

        unsigned long lastTime;
        double lastError;
        double error;

        double tolerance;

        double totalError;
        double minimumIntegral;
        double maximumIntegral;


        // Constructor with kp and kd
        PIDController(double kP, double kD);

        //Constructor with all
        PIDController(double kP, double kI, double kD);
        
        // Default constructor, sets to 0
        PIDController();
        
        // Constructor with just kp
        PIDController(double kP);

        // Calculate the control output, should be called periodically
        double calculate(double measure);

        // Set tolerance for "at setpoint" determination
        void setTolerance(double tolerance);

        // Check if controller is at the setpoint using tolerance
        bool atSetpoint();

        //Updates setpoint
        void setSetpoint(double setpoint);

        //Sets the minimum and maximum for integral
        void setIntegralRange(double min, double max);

        //Resets the PIDController
        void reset();
};

#endif
