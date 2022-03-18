#include "Teleop.h"

Teleop::Teleop(SwerveDrive* drive) {
    m_drive = drive;
}

void Teleop::Init() {
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