#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include "Constants.h"

class SwerveDrive {
    rev::CANSparkMax m_leftFrontDrive{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftFrontAngle{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftRearDrive{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftRearDrive{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFrontDrive{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFrontDrive{6, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightRearDrive{7, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightRearDrive{8, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

    //std::shared_ptr<rev::CANSparkMax> m_leftFrontDrive = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::LEFT_FRONT.DRIVE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

    //rev::SparkMaxRelativeEncoder m_leftFrontDriveEncoder = m_leftFrontDrive.GetEncoder();
};