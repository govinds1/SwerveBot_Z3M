package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;

public class WheelModuleSubsystem extends SubsystemBase {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_angleMotor;

    private RelativeEncoder m_driveEncoder;
    private Encoder m_angleEncoder;

    private SparkMaxPIDController m_drivePIDController;
    private PIDController m_anglePIDController;

    // define a zeroAngle that corresponds to the encoder value of the wheel facing forwards
    private int m_angleEncoderZero;

    SwerveModuleState m_desiredState;
    SwerveModuleState m_trueState;

    public WheelModuleSubsystem(int[] motorIDs, int[] angleEncoderIDs, int angleEncoderZero) {
        m_driveMotor = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
        m_angleMotor = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_angleEncoder = new Encoder(angleEncoderIDs[0], angleEncoderIDs[1]);

        
        m_drivePIDController = m_driveMotor.getPIDController();
        SetDrivePID(Calibrations.DRIVE_PID_VALUES[0], Calibrations.DRIVE_PID_VALUES[1], Calibrations.DRIVE_PID_VALUES[2]);
        m_anglePIDController = new PIDController(Calibrations.ANGLE_PID_VALUES[0], Calibrations.ANGLE_PID_VALUES[1], Calibrations.ANGLE_PID_VALUES[2]);

        // tolerances in radians
        m_anglePIDController.setTolerance(0.005, 0.01);
        
        m_driveEncoder.setPositionConversionFactor(Calibrations.DRIVE_ENCODER_FEET_PER_TICK);
        m_driveEncoder.setVelocityConversionFactor(Calibrations.DRIVE_ENCODER_FEET_PER_TICK / 60);
        
        m_angleEncoderZero = angleEncoderZero;
        m_angleEncoder.setDistancePerPulse(Calibrations.ANGLE_ENCODER_RADIAN_PER_PULSE);
    }

    public void init() {

    }

    @Override
    public void periodic() {
      // Calculate angle PID output using desired state angle and set motor output
      m_angleMotor.set(m_anglePIDController.calculate(GetAngle().getRadians(), m_desiredState.angle.getRadians()));

      // Set drive velocity reference
      m_drivePIDController.setReference(Units.metersToFeet(m_desiredState.speedMetersPerSecond), ControlType.kVelocity);

      // Calculate and set true state
      SetTrueState();
    }

    public void SetDrivePID(double p, double i, double d) {
        m_drivePIDController.setP(p);
        m_drivePIDController.setI(i);
        m_drivePIDController.setD(d);
    }

    public void ResetState() {
        SetDesiredState(0, 0);
    }

    public void ResetAngle() {
        m_desiredState.angle = new Rotation2d(0);
    }

    public void CalibrateAngle() {
        // Call this function when the wheel faces forward, will set current encoder ticks as the zero variable
        m_angleEncoderZero = m_angleEncoder.get(); // should be in raw ticks (without conversion) when using quadrature encoder
    }

    // Set the desired state of the swerve module -> setpoints from drive input
    public void SetDesiredState(SwerveModuleState newState) {
        m_desiredState = SwerveModuleState.optimize(newState, GetAngle());
    }

    public void SetDesiredState(double feetPerSec, double radians) {
        SetDesiredState(feetPerSec, new Rotation2d(radians));
    }

    public void SetDesiredState(double feetPerSec, Rotation2d radians) {
        SetDesiredState(new SwerveModuleState(feetPerSec, radians));
    }

    // Set the real state of the swerve module -> for odometry purposes
    public void SetTrueState() {
        m_trueState.speedMetersPerSecond = GetSpeed();
        
    }

    // Wheel angle of 0 (from a getter function) is considered to be facing forwards
    // Wheel azimuth in encoder pulses
    public int GetRawAngle() {
        return m_angleEncoder.get() - m_angleEncoderZero;
    }

    // Wheel azimuth in radians
    public Rotation2d GetAngle() {
        return new Rotation2d(m_angleEncoder.getDistance() - (m_angleEncoderZero * m_angleEncoder.getDistancePerPulse()));
    }

    // Wheel drive speed in feet per second
    public double GetSpeed() {
        return m_driveEncoder.getVelocity();
    }

    // Wheel drive distance in feet
    public double GetDistance() {
        return m_driveEncoder.getPosition();
    }

    public SwerveModuleState GetDesiredState() {
        return m_desiredState;
    }

    public SwerveModuleState GetTrueState() {
        return m_trueState;
    }
}
