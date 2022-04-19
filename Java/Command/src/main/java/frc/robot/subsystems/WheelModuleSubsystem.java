package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelModuleSubsystem extends SubsystemBase {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_angleMotor;

    private RelativeEncoder m_driveEncoder;
    private Encoder m_angleEncoder;

    public WheelModuleSubsystem(int[] motorIDs, int[] angleEncoderIDs) {
        m_driveMotor = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
        m_angleMotor = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_angleEncoder = new Encoder(angleEncoderIDs[0], angleEncoderIDs[1]);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
