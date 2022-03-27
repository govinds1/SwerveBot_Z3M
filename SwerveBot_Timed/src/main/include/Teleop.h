#pragma once

#include <frc/XboxController.h>
#include "SwerveDrive.h"

class Teleop {
    frc::XboxController m_driverController{0};
    std::shared_ptr<SwerveDrive> m_drive;
    std::shared_ptr<PathManager> m_pathManager;

    public:
    Teleop(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<PathManager> pathManager);
    void Init();
    void Periodic();
};