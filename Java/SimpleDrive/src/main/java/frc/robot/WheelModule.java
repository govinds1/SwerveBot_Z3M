package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


// Uses two Falcon motors (TalonFX controllers) to simulate a swerve WheelModule
// Simulate a PID Controller
public class WheelModule {
    TalonFX m_driveMotor;
    TalonFX m_angleMotor;

    // Stores speed and angle to update to the motors periodically
    double m_desiredVelocity = 0;
    double m_desiredAngle = 0;

    // Encoder conversions
    double m_driveVelocityConversion = (1 / (2048 * 10.71)) * 600; // 1 / (ticksPerRotation * gearRatio) * 100msToSec
    double m_anglePositionConversion = 1 / (2048 * 10.71 * 2 * Math.PI); // 1 / (ticksPerRotation * gearRatio * 2PI)

    // Stores the current PID output for each motor
    double m_currentDriveAcceleration = 0; // feet per second^2
    double m_currentAngleVelocity = 0; // radians per second

    // Max Velocities
    double m_driveVelocityMaximum = 5.0; // feet per second
    double m_angleVelocityMaximum = 12.48; // radians per second

    // PID Parameters
    double m_driveP = 0;
    double m_driveI = 0;
    double m_driveD = 0;
    double m_driveFF = 0;

    double m_angleP = 0;
    double m_angleI = 0;
    double m_angleD = 0;
    double m_angleFF = 0;

    // PID accumulated error
    double m_driveAccumError = 0;
    double m_angleAccumError = 0;

    // Setpoint thresholds
    double m_driveFeetPerSecThreshold = 0.01;
    double m_angleRadianThreshold = 0.01;

    // Add any motor control features to simulate

    // Stores angle motor encoder ticks where the wheel faces forward
    int m_angleEncoderZero;

    // Delta time
    double dt = 0.02;


    public WheelModule(int driveID, int angleID, int angleEncoderZero) {
        m_driveMotor = new TalonFX(driveID);
        m_angleMotor = new TalonFX(angleID);

        m_angleEncoderZero = angleEncoderZero;
    }

    public void Periodic() {
        // boring and easy :]
        // m_driveMotor.set(TalonFXControlMode.Velocity, m_desiredSpeed);
        // m_angleMotor.set(TalonFXControlMode.Position, m_desiredAngle);

        // general PID pseudocode for some variable x
        // error = desired_x - current_x
        // next_error = desired_x - (current_x_dot + current_x)
        // next_x_dot = kp * error + ki * accum_error + kd * (next_error - error)
        // accum_error = accum_error + error
        
        // Drive Velocity PID Control
        // x is the velocity (so x_dot is acceleration)
        double currentVelocity = GetVelocity();
        double velocityError = m_desiredVelocity - currentVelocity;
        double nextVelocityError = m_desiredVelocity - (m_currentDriveAcceleration * dt + currentVelocity);
        double driveAcceleration = m_driveP * velocityError + m_driveI * m_driveAccumError + m_driveD * (nextVelocityError - velocityError);
        m_driveAccumError = m_driveAccumError + velocityError;

        // Angle Position PID Control
        // x is the position
        double currentAngle = GetAngle();
        double angleError = m_desiredAngle - currentAngle;
        double nextAngleError = m_desiredAngle - (m_currentAngleVelocity * dt + currentAngle);
        double angleVelocity = m_angleP * angleError + m_angleI * m_angleAccumError + m_angleD * (nextAngleError - angleError); 

        // Convert Drive acceleration and set motor
        m_currentDriveAcceleration = driveAcceleration;
        double currentDriveOutput = m_driveMotor.getMotorOutputPercent();
        double driveOutput = currentDriveOutput + (driveAcceleration * dt) / m_driveVelocityMaximum;
        m_driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput);

        // Convert Angle velocity and set motor
        m_currentAngleVelocity = angleVelocity;
        double angleOutput = angleVelocity / m_angleVelocityMaximum;
        m_angleMotor.set(TalonFXControlMode.PercentOutput, angleOutput);

    }

    // speed in feet per second
    // angle in radians
    public void SetDesiredState(double velocity, double angle) {
        m_desiredVelocity = velocity;
        m_desiredAngle = angle;
    }

    public double GetVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * m_driveVelocityConversion;
    }

    public double GetAngle() {
        return m_angleMotor.getSelectedSensorPosition() * m_anglePositionConversion;
    }
}
