#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/ADXRS450_Gyro.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

#include "WheelModule.h"
#include "PathManager.h"

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

    // rev::SparkMaxRelativeEncoder m_leftFrontDriveEncoder = m_leftFrontDrive->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_leftFrontAngleEncoder = m_leftFrontAngle->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_leftRearDriveEncoder = m_leftRearDrive->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_leftRearAngleEncoder = m_leftRearAngle->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_rightFrontDriveEncoder = m_rightFrontDrive->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_rightFrontAngleEncoder = m_rightFrontAngle->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_rightRearDriveEncoder = m_rightRearDrive->GetEncoder();
    // rev::SparkMaxRelativeEncoder m_rightRearAngleEncoder = m_rightRearAngle->GetEncoder();

    // rev::SparkMaxPIDController m_leftFrontDrivePIDController =  m_leftFrontDrive->GetPIDController();
    // rev::SparkMaxPIDController m_leftFrontAnglePIDController =  m_leftFrontAngle->GetPIDController();
    // rev::SparkMaxPIDController m_leftRearDrivePIDController =  m_leftRearDrive->GetPIDController();
    // rev::SparkMaxPIDController m_leftRearAnglePIDController =  m_leftRearAngle->GetPIDController();
    // rev::SparkMaxPIDController m_rightFrontDrivePIDController =  m_rightFrontDrive->GetPIDController();
    // rev::SparkMaxPIDController m_rightFrontAnglePIDController =  m_rightFrontAngle->GetPIDController();
    // rev::SparkMaxPIDController m_rightRearDrivePIDController =  m_rightRearDrive->GetPIDController();
    // rev::SparkMaxPIDController m_rightRearAnglePIDController =  m_rightRearAngle->GetPIDController();

    WheelModule* m_leftFront = new WheelModule(m_leftFrontDrive, m_leftFrontAngle, m_leftFrontDriveEncoder, m_leftFrontAngleEncoder, m_leftFrontDrivePIDController, m_leftFrontAnglePIDController, ENCODER_ZEROS::LEFT_FRONT);
    WheelModule* m_leftRear = new WheelModule(m_leftRearDrive, m_leftRearAngle, m_leftRearDriveEncoder, m_leftRearAngleEncoder, m_leftRearDrivePIDController, m_leftRearAnglePIDController, ENCODER_ZEROS::LEFT_REAR);
    WheelModule* m_rightFront = new WheelModule(m_rightFrontDrive, m_rightFrontAngle, m_rightFrontDriveEncoder, m_rightFrontAngleEncoder, m_rightFrontDrivePIDController, m_rightFrontAnglePIDController, ENCODER_ZEROS::RIGHT_FRONT);
    WheelModule* m_rightRear = new WheelModule(m_rightRearDrive, m_rightRearAngle, m_rightRearDriveEncoder, m_rightRearAngleEncoder, m_rightRearDrivePIDController, m_rightRearAnglePIDController, ENCODER_ZEROS::RIGHT_REAR);

    // Translations are x and y distance from center of robot
    // x is positive towards the front of the robot
    // y is positive towards the left of the robot
    frc::Translation2d m_leftFrontLocation{DIMENSIONS::WHEELBASE_LENGTH / 2.0, DIMENSIONS::WHEELBASE_WIDTH / 2.0};
    frc::Translation2d m_leftRearLocation{-DIMENSIONS::WHEELBASE_LENGTH / 2.0, DIMENSIONS::WHEELBASE_WIDTH / 2.0};
    frc::Translation2d m_rightFrontLocation{DIMENSIONS::WHEELBASE_LENGTH / 2.0, -DIMENSIONS::WHEELBASE_WIDTH / 2.0};
    frc::Translation2d m_rightRearLocation{-DIMENSIONS::WHEELBASE_LENGTH / 2.0, -DIMENSIONS::WHEELBASE_WIDTH / 2.0};

    frc::SwerveDriveKinematics<4> m_kinematics{m_leftFrontLocation, m_leftRearLocation, m_rightFrontLocation, m_rightRearLocation};
    frc::ChassisSpeeds m_desiredSpeeds;
    frc::ChassisSpeeds m_trueSpeeds;

    std::shared_ptr<PathManager> m_pathManager;

    frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, GetAngle()};
    // Add Gyro
    frc::ADXRS450_Gyro m_gyro;
    bool fieldRelative = false;


    public:
    SwerveDrive(std::shared_ptr<PathManager> pathManager);
    
    void Init(bool autonomous = false, frc::Pose2d initialPose = frc::Pose2d(0_m, 0_m, 0_rad));
    void Periodic();
    void Drive(double forward, double right, double turn);
    void Drive(double forward, double right, double turn, bool driveFieldRelative);
    double FollowTrajectory(units::time::second_t currentTime, std::string trajectoryName);
    double FollowTrajectory(units::time::second_t currentTime, std::string startPoseName, std::string endPoseName);

    void ResetSpeeds();
    void CalibrateWheelsManually();
    void CalibrateWheelsAuto(units::velocity::feet_per_second_t desiredVX);
    void SetDesiredSpeeds(double vx, double vy, double omega);
    void SetDesiredSpeeds(units::velocity::feet_per_second_t vx, units::velocity::feet_per_second_t vy, units::angular_velocity::radians_per_second_t omega);
    void SetDesiredSpeeds(frc::ChassisSpeeds newSpeeds);
    void SetTrueSpeeds();
    void SetWheelStates();
    void SetFieldRelative(bool fieldRel);
    void SetPose(frc::Pose2d newPose);
    void SetPose(double x, double y);
    void SetPose(units::meter_t x, units::meter_t y);
    void ResetGyro();

    frc::Rotation2d GetAngle();
    frc::Pose2d GetPose();
    frc::ChassisSpeeds GetDesiredSpeeds();
    frc::ChassisSpeeds GetTrueSpeeds();

};