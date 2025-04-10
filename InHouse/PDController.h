#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

class PDController {
    public:
        double kp;
        double kd;

        double setpoint;

        unsigned long lastTime;
        double lastError;

        double tolerance;

        // Constructor with kp and kd
        PDController(double kp, double kd);
        
        // Default constructor
        PDController();
        
        // Constructor with just kp
        PDController(double kp);

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