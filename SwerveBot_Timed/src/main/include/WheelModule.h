#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>

class WheelModule {
    std::shared_ptr<rev::CANSparkMax> m_driveMotor;
    std::shared_ptr<rev::CANSparkMax> m_angleMotor;


    frc::SwerveModuleState m_state;
    frc::Rotation2d zeroRotation;

    public: 
    WheelModule(std::shared_ptr<rev::CANSparkMax> driveMotor, std::shared_ptr<rev::CANSparkMax> angleMotor);
};