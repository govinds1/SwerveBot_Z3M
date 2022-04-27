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

    // define a zeroAngle that corresponds to the encoder value of the wheel facing forwards
    // unnecessary when using the SparkMaxRelativeEncoder from the built-in NEO hall effect sensor
    int m_angleEncZero = 0;

    frc::SwerveModuleState m_desiredState;
    frc::SwerveModuleState m_trueState;

    public: 
    WheelModule(std::shared_ptr<rev::CANSparkMax> driveMotor, std::shared_ptr<rev::CANSparkMax> angleMotor, 
            std::shared_ptr<rev::SparkMaxRelativeEncoder> driveEncoder, std::shared_ptr<rev::SparkMaxRelativeEncoder> angleEncoder,
            std::shared_ptr<rev::SparkMaxPIDController> drivePIDController, std::shared_ptr<rev::SparkMaxPIDController> anglePIDController,
            int angleEncZero);

    void Init();
    void Periodic();

    void ResetState();
    void ResetAngle();
    void CalibrateAngle();
    void SetDesiredState(frc::SwerveModuleState newState);
    void SetDesiredState(double feetPerSec, double radians);
    void SetDesiredState(double feetPerSec, frc::Rotation2d radians);
    void SetTrueState();
    void SetDrivePID(double p = 0.0, double i = 0.0, double d = 0.0);
    void SetAnglePID(double p = 0.0, double i = 0.0, double d = 0.0);

    int GetRawAngle();
    units::angle::radian_t GetAngle();
    units::velocity::feet_per_second_t GetSpeed();
    units::length::foot_t GetDistance();
    const frc::SwerveModuleState& GetDesiredState();
    const frc::SwerveModuleState& GetTrueState();
};