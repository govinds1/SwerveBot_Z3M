#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

#include "Constants.h"

class Turret {
    std::shared_ptr<WPI_TalonFX> m_turret = std::make_shared<WPI_TalonFX>(MOTOR_CAN_ID::TURRET);

    public:
    Turret();
    void Init();
    void Periodic();

    void SetTurretPID(double p = 0.0, double i = 0.0, double d = 0.0, double f = 0.0);

};