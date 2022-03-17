#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/interfaces/Gyro.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include "Constants.h"
#include "WheelModule.h"

class SwerveDrive {
    std::shared_ptr<rev::CANSparkMax> m_leftFrontDrive = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::LEFT_FRONT.DRIVE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_leftFrontAngle = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::LEFT_FRONT.ANGLE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_leftRearDrive = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::LEFT_REAR.DRIVE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_leftRearAngle = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::LEFT_REAR.ANGLE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_rightFrontDrive = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::RIGHT_FRONT.DRIVE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_rightFrontAngle = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::RIGHT_FRONT.ANGLE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_rightRearDrive = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::RIGHT_REAR.DRIVE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    std::shared_ptr<rev::CANSparkMax> m_rightRearAngle = std::make_shared<rev::CANSparkMax>(MOTOR_CAN_ID::RIGHT_REAR.ANGLE_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_leftFrontDriveEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_leftFrontDrive->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_leftFrontAngleEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_leftFrontAngle->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_leftRearDriveEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_leftRearDrive->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_leftRearAngleEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_leftRearAngle->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_rightFrontDriveEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_rightFrontDrive->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_rightFrontAngleEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_rightFrontAngle->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_rightRearDriveEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_rightRearDrive->GetEncoder());
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_rightRearAngleEncoder = std::make_shared<rev::SparkMaxRelativeEncoder>(m_rightRearAngle->GetEncoder());

    std::shared_ptr<rev::SparkMaxPIDController> m_leftFrontDrivePIDController = std::make_shared<rev::SparkMaxPIDController>(m_leftFrontDrive->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_leftFrontAnglePIDController = std::make_shared<rev::SparkMaxPIDController>(m_leftFrontAngle->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_leftRearDrivePIDController = std::make_shared<rev::SparkMaxPIDController>(m_leftRearDrive->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_leftRearAnglePIDController = std::make_shared<rev::SparkMaxPIDController>(m_leftRearAngle->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_rightFrontDrivePIDController = std::make_shared<rev::SparkMaxPIDController>(m_rightFrontDrive->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_rightFrontAnglePIDController = std::make_shared<rev::SparkMaxPIDController>(m_rightFrontAngle->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_rightRearDrivePIDController = std::make_shared<rev::SparkMaxPIDController>(m_rightRearDrive->GetPIDController());
    std::shared_ptr<rev::SparkMaxPIDController> m_rightRearAnglePIDController = std::make_shared<rev::SparkMaxPIDController>(m_rightRearAngle->GetPIDController());

    WheelModule m_leftFront{m_leftFrontDrive, m_leftFrontAngle, m_leftFrontDriveEncoder, m_leftFrontAngleEncoder, m_leftFrontDrivePIDController, m_leftFrontAnglePIDController};
    WheelModule m_leftRear{m_leftRearDrive, m_leftRearAngle, m_leftRearDriveEncoder, m_leftRearAngleEncoder, m_leftRearDrivePIDController, m_leftRearAnglePIDController};
    WheelModule m_rightFront{m_rightFrontDrive, m_rightFrontAngle, m_rightFrontDriveEncoder, m_rightFrontAngleEncoder, m_rightFrontDrivePIDController, m_rightFrontAnglePIDController};
    WheelModule m_rightRear{m_rightRearDrive, m_rightRearAngle, m_rightRearDriveEncoder, m_rightRearAngleEncoder, m_rightRearDrivePIDController, m_rightRearAnglePIDController};

    // Translations are x and y distance from center of robot
    // x is positive towards the front of the robot
    // y is positive towards the left of the robot
    frc::Translation2d m_leftFrontLocation{DIMENSIONS::WHEELBASE_LENGTH / 2.0, DIMENSIONS::WHEELBASE_WIDTH / 2.0};
    frc::Translation2d m_leftRearLocation{-DIMENSIONS::WHEELBASE_LENGTH / 2.0, DIMENSIONS::WHEELBASE_WIDTH / 2.0};
    frc::Translation2d m_rightFrontLocation{DIMENSIONS::WHEELBASE_LENGTH / 2.0, -DIMENSIONS::WHEELBASE_WIDTH / 2.0};
    frc::Translation2d m_rightRearLocation{-DIMENSIONS::WHEELBASE_LENGTH / 2.0, -DIMENSIONS::WHEELBASE_WIDTH / 2.0};

    frc::SwerveDriveKinematics<4> m_kinematics{m_leftFrontLocation, m_leftRearLocation, m_rightFrontLocation, m_rightRearLocation};
    frc::ChassisSpeeds m_desiredSpeeds;


    // Add SwerveDriveOdometry
    // Add Gyro
    bool fieldRelative = false;


    public:
    SwerveDrive();
    
    void Init();
    void Periodic();
    void Drive(double forward, double right, double turn);

    void ResetSpeeds();
    void SetSpeeds(double vx, double vy, double omega);
    void SetSpeeds(units::velocity::feet_per_second_t vx, units::velocity::feet_per_second_t vy, units::angular_velocity::radians_per_second_t omega);
    void SetWheelStates();
    void SetFieldRelative(bool fieldRel);

    frc::Rotation2d GetAngle();

};