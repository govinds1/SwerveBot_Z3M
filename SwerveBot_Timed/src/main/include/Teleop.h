#pragma once

#include "SwerveDrive.h"
#include <frc/XboxController.h>

class Teleop {
    frc::XboxController m_driverController{0};
    SwerveDrive* m_drive;

    public:
    Teleop(SwerveDrive* drive);
    void Init();
    void Periodic();
};