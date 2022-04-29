package frc.robot.utils;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kFF;
    private double dt;
    
    private double setpoint;
    private double lastValue;
    private double accumError;
    private double thresh;

    // Initialize PIDController with given PID constants
    public PIDController(double P, double I, double D, double FF) {
        kP = P;
        kI = I;
        kD = D;
        kFF = FF;
        dt = 0.02;
        setpoint = 0;
        accumError = 0;
        thresh = 0.01;
        lastValue = 0;
    }

    public void SetPeriod(double period) {
        dt = period;
    }

    public double GetPeriod() {
        return dt;
    }

    // Calculates the PID output from a given current value, target value, and derivative of the current value
    // Stores the target as the setpoint of the PID Controller
    public double Calculate(double current, double target, double current_dot) {
        setpoint = target;
        return Calculate(current, current_dot);
    }

    // Calculates the PID output from a given current value and derivative of the current value
    // Uses the stored setpoint from a previous set
    public double Calculate(double current, double current_dot) {
        // general PID pseudocode for some variable x
        // error = target_x - current_x
        // next_error = target_x - (current_x_dot * delta_t + current_x)
        // next_x_dot = kp * error + ki * accum_error + kd * (next_error - error) + kf * target_x
        // accum_error = accum_error + error
        // IF: you ensure that Calculate is run every cycle, then you can calculate current_dot from lastVal and current
        double error = setpoint - current;
        double nextError = setpoint - (current_dot * dt + current);
        double output = kP * error + kI * accumError + kD * (nextError - error) + kFF * setpoint;
        accumError = accumError + error;
        lastValue = current;

        return output;
    }

    // Returns the difference between the stored setpoint and last given measurement
    public double GetError() {
        return setpoint - lastValue;
    }

    // Returns whether the given current value is within the threshold of the stored setpoint
    public boolean AtSetpoint(double current) {
        return (Math.abs(setpoint - current) < thresh);
    }
    
    // Returns whether the given current value is within the threshold of the given target value
    // Stores the target as the setpoint of the PID Controller
    public boolean AtSetpoint(double current, double target) {
        setpoint = target;
        return AtSetpoint(current);
    }
    
    // Returns whether the last measurement given to the PIDController is within the threshold of the stored setpoint
    public boolean AtSetpoint() {
        return AtSetpoint(lastValue);
    }

    
    // resets accumulated error
    public void Reset() {
        accumError = 0;
    }
    
    public void SetP(double p) {
        kP = p;
    }
    
    public void SetI(double i) {
        kI = i;
    }
    
    public void SetD(double d) {
        kD = d;
    }
    
    public void SetFF(double ff) {
        kFF = ff;
    }

    // Sets the setpoint for future calcualtion
    public void SetSetpoint(double target) {
        setpoint = target;
    }

    public void SetThreshold(double threshold) {
        thresh = threshold;
    }
}
