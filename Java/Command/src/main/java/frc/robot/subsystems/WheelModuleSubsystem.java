package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;

public class WheelModuleSubsystem extends SubsystemBase {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_angleMotor;

    private RelativeEncoder m_driveEncoder;
    private Encoder m_angleEncoder;

    private SparkMaxPIDController m_drivePIDController;
    private PIDController m_anglePIDController;

    public WheelModuleSubsystem(int[] motorIDs, int[] angleEncoderIDs) {
        m_driveMotor = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
        m_angleMotor = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_angleEncoder = new Encoder(angleEncoderIDs[0], angleEncoderIDs[1]);

        m_drivePIDController = m_driveMotor.getPIDController();
        SetDrivePID(Calibrations.DRIVE_PID_VALUES[0], Calibrations.DRIVE_PID_VALUES[1], Calibrations.DRIVE_PID_VALUES[2]);
        m_anglePIDController = new PIDController(Calibrations.ANGLE_PID_VALUES[0], Calibrations.ANGLE_PID_VALUES[1], Calibrations.ANGLE_PID_VALUES[2]);

        m_driveEncoder.setPositionConversionFactor(RobotMap.DRIVE_ENCODER_FEET_PER_TICK);
        m_driveEncoder.setVelocityConversionFactor(RobotMap.DRIVE_ENCODER_FEET_PER_TICK / 60);
        m_angleEncoder.setDistancePerPulse(RobotMap.ANGLE_ENCODER_RADIAN_PER_PULSE);
    }

    public void init() {

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void SetDrivePID(double p, double i, double d) {
        m_drivePIDController.setP(p);
        m_drivePIDController.setI(i);
        m_drivePIDController.setD(d);
    }
}
