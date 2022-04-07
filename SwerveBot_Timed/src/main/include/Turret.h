#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "ctre/Phoenix.h"

#include "SwerveDrive.h"
#include "Constants.h"

class Turret {
    std::shared_ptr<WPI_TalonFX> m_turret = std::make_shared<WPI_TalonFX>(MOTOR_CAN_ID::TURRET);
    std::shared_ptr<WPI_TalonFX> m_shooter = std::make_shared<WPI_TalonFX>(MOTOR_CAN_ID::SHOOTER);
    std::shared_ptr<SwerveDrive> m_drive;
    frc::Rotation2d startAngle;
    std::shared_ptr<nt::NetworkTable> limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    public:
    Turret(std::shared_ptr<SwerveDrive> drive);
    void Init();
    void Periodic();

    bool ShootAtHub();
    bool GoToAngle(frc::Rotation2d angle);
    void TurnAngle(frc::Rotation2d deltaAngle);
    bool AtAngle(frc::Rotation2d setpointAngle);
    frc::Rotation2d SpeedOffset(units::velocity::feet_per_second_t &vball);
    frc::Rotation2d SpeedOffset(units::velocity::feet_per_second_t &vball, frc::Rotation2d theta);
    frc::Rotation2d FromZero(frc::Rotation2d theta, double y_val);
    void SetTurretPID(double p = 0.0, double i = 0.0, double d = 0.0, double f = 0.0);
    void SetShooterPID(double p = 0.0, double i = 0.0, double d = 0.0, double f = 0.0);
    frc::Rotation2d GetAngle();
    frc::Rotation2d EncoderToAngle(double encoderUnits);
    double AngleToEncoder(frc::Rotation2d angle);
    frc::Rotation2d NormalizeAngle(frc::Rotation2d angle);
    void RunShooter(units::velocity::feet_per_second_t ballLinearLaunchVelocity);
    void RunShooter(units::angular_velocity::revolutions_per_minute_t rpm);
    units::angular_velocity::revolutions_per_minute_t GetRPM();

};