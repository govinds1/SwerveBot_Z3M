#include "Teleop.h"

Teleop::Teleop(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<PathManager> pathManager) {
    m_drive = drive;
    m_pathManager = pathManager;
}

void Teleop::Init() {
    m_pathManager->SetEnabled(true);
    m_drive->Init();
}

void Teleop::Periodic() {
    m_drive->Drive(m_driverController.GetLeftY(), m_driverController.GetLeftX(), m_driverController.GetRightX());
    if (m_driverController.GetAButton()) {
        m_drive->SetFieldRelative(true);
    } else {
        m_drive->SetFieldRelative(false);
    }

    m_drive->Periodic();
}