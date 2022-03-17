#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

class WheelModule {
    std::shared_ptr<rev::CANSparkMax> m_driveMotor;
    std::shared_ptr<rev::CANSparkMax> m_angleMotor;

    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_driveEncoder;
    std::shared_ptr<rev::SparkMaxRelativeEncoder> m_angleEncoder; // MOST LIKELY WE WILL USE A QUADRATURE ENCODER FOR ANGLE RATHER THAN BUILT-IN HALL EFFECT SENSOR
    std::shared_ptr<rev::SparkMaxPIDController> m_drivePIDController;
    std::shared_ptr<rev::SparkMaxPIDController> m_anglePIDController;


    frc::SwerveModuleState m_state;

    public: 
    WheelModule(std::shared_ptr<rev::CANSparkMax> driveMotor, std::shared_ptr<rev::CANSparkMax> angleMotor, 
            std::shared_ptr<rev::SparkMaxRelativeEncoder> driveEncoder, std::shared_ptr<rev::SparkMaxRelativeEncoder> angleEncoder,
            std::shared_ptr<rev::SparkMaxPIDController> drivePIDController, std::shared_ptr<rev::SparkMaxPIDController> anglePIDController);

    void Init();
    void Periodic();

    void ResetState();
    void ResetAngle();
    void SetState(frc::SwerveModuleState newState);
    void SetState(double feetPerSec, double radians);
    void SetState(double feetPerSec, frc::Rotation2d radians);
    void SetDrivePID(double p = 0.0, double i = 0.0, double d = 0.0);
    void SetAnglePID(double p = 0.0, double i = 0.0, double d = 0.0);

    units::angle::radian_t GetAngle();
    units::velocity::feet_per_second_t GetSpeed();
    units::length::foot_t GetDistance();
};