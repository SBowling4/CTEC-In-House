#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
    public:
        double kp;
        double kd;
        double ki;

        double setpoint;

        unsigned long lastTime;
        double lastError;

        double tolerance;

        double integral;


        // Constructor with kp and kd
        PIDController(double kp, double kd);

        //Constructor with all
        PIDController(double kp, double ki, double kd);
        
        // Default constructor
        PIDController();
        
        // Constructor with just kp
        PIDController(double kp);

        // Calculate the control output
        double calculate(double measure);

        // Set tolerance for "at setpoint" determination
        void setTolerance(double tolerance);

        // Check if we're at the setpoint
        bool atSetpoint();

        //Updates setpoint
        void setSetpoint(double setpoint);
};

#endif
