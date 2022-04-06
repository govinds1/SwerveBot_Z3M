#pragma once

#include <frc/XboxController.h>
#include "SwerveDrive.h"
#include "Turret.h"

class Teleop {
    frc::XboxController m_driverController{0};
    std::shared_ptr<SwerveDrive> m_drive;
    std::shared_ptr<PathManager> m_pathManager;
    std::shared_ptr<Turret> m_turret;

    public:
    Teleop(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<PathManager> pathManager, std::shared_ptr<Turret> turret);
    void Init();
    void Periodic();
};