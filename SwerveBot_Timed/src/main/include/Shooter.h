#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

class Shooter {
    std::shared_ptr<WPI_TalonFX> m_shooter = std::make_shared<WPI_TalonFX>(MOTOR_CAN_ID::SHOOTER);

    public:
    Shooter();
    void Init();
    void Periodic();

    void SetShooterPID(double p = 0.0, double i = 0.0, double d = 0.0, double f = 0.0);

};