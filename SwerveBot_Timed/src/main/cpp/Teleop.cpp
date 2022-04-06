#include "Teleop.h"

Teleop::Teleop(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<PathManager> pathManager, std::shared_ptr<Turret> turret) {
    m_drive = drive;
    m_turret = turret;
    m_pathManager = pathManager;
}

void Teleop::Init() {
    m_pathManager->SetEnabled(true);
    m_drive->Init();
    m_turret->Init();
}

void Teleop::Periodic() {
    m_drive->Drive(m_driverController.GetLeftY(), m_driverController.GetLeftX(), m_driverController.GetRightX());
    if (m_driverController.GetAButton()) {
        m_drive->SetFieldRelative(true);
    } else {
        m_drive->SetFieldRelative(false);
    }

    m_drive->Periodic();
    m_turret->Periodic();
}