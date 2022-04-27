package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class WheelModule {
    CANSparkMax m_driveMotor;
    CANSparkMax m_angleMotor;

    double desiredSpeed;
    double desiredAngle;

    public WheelModule(int driveID, int angleID) {
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        m_angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
    }
}
