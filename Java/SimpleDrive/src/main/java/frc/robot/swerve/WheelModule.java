package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.utils.PIDController;


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

    PIDController m_driveVelocityPID = new PIDController(m_driveP, m_driveI, m_driveD, m_driveFF);
    PIDController m_anglePositionPID = new PIDController(m_angleP, m_angleI, m_angleD, m_angleFF);

    // Setpoint thresholds
    double m_driveFeetPerSecThreshold = 0.01;
    double m_angleRadianThreshold = 0.01;

    // Add any motor control features to simulate

    // Stores angle motor encoder ticks where the wheel faces forward
    int m_angleEncoderZero;


    public WheelModule(int driveID, int angleID, int angleEncoderZero) {
        m_driveMotor = new TalonFX(driveID);
        m_angleMotor = new TalonFX(angleID);

        m_angleEncoderZero = angleEncoderZero;

        m_driveVelocityPID.SetThreshold(m_driveFeetPerSecThreshold);
        m_anglePositionPID.SetThreshold(m_angleRadianThreshold);
    }

    public void Periodic() {
        // boring and easy :]
        // m_driveMotor.set(TalonFXControlMode.Velocity, m_desiredSpeed);
        // m_angleMotor.set(TalonFXControlMode.Position, m_desiredAngle);
        
        // Drive Velocity PID Control
        double driveAcceleration = m_driveVelocityPID.Calculate(GetVelocity(), m_desiredVelocity, m_currentDriveAcceleration);

        // Angle Position PID Control
        double angleVelocity = m_anglePositionPID.Calculate(GetAngle(), m_desiredAngle, m_currentAngleVelocity);

        // Convert Drive acceleration and set motor
        m_currentDriveAcceleration = driveAcceleration;
        double currentDriveOutput = m_driveMotor.getMotorOutputPercent();
        double driveOutput = currentDriveOutput + (driveAcceleration * m_driveVelocityPID.GetPeriod()) / m_driveVelocityMaximum;
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

    // drive is in [0, 1]
    // angle is in [-3.14, 3.14]
    public void Drive(double drive, double angle) {
        // Add angle offset
        angle += m_angleEncoderZero * m_anglePositionConversion;

        // Optimize angle
            // the wheel should not turn more than 90 degrees
        double currentAngle = GetAngle();
        while (Math.abs(currentAngle - angle) > (Math.PI / 2.0)) {
            // flip angle 180 degrees and reverse speed
            drive = -drive;
            if (currentAngle < angle) {
                angle -= Math.PI;
            } else {
                angle += Math.PI;
            }
        }

        // Convert drive to velocity units
        double velocity = drive * m_driveVelocityMaximum;

        // Set desired state
        SetDesiredState(velocity, angle);
    }

    // Returns wheel velocity in feet per second
    public double GetVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * m_driveVelocityConversion;
    }

    // Returns wheel angle in radians
    public double GetAngle() {
        return m_angleMotor.getSelectedSensorPosition() * m_anglePositionConversion;
    }
}
