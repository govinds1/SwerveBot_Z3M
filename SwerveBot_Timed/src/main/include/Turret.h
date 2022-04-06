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
    std::shared_ptr<SwerveDrive> m_drive;
    frc::Rotation2d startAngle;
    std::shared_ptr<nt::NetworkTable> limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    public:
    Turret(std::shared_ptr<SwerveDrive> drive);
    void Init();
    void Periodic();

    bool AlignToHub();
    bool GoToAngle(frc::Rotation2d angle);
    void TurnAngle(frc::Rotation2d deltaAngle);
    bool AtAngle(frc::Rotation2d setpointAngle);
    void SetTurretPID(double p = 0.0, double i = 0.0, double d = 0.0, double f = 0.0);
    frc::Rotation2d GetAngle();
    frc::Rotation2d EncoderToAngle(double encoderUnits);
    double AngleToEncoder(frc::Rotation2d angle);
    frc::Rotation2d NormalizeAngle(frc::Rotation2d angle);

};