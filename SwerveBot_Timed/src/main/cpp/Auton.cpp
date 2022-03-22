#include "Auton.h"

Auton::Auton(std::shared_ptr<SwerveDrive> swerveDrive) {
    m_swerveDrive = swerveDrive;
    frc::SmartDashboard::PutStringArray("Auto List", m_autoList);
    m_stateStartTime = 0_s;
}

void Auton::Init() {
    m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", "");
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
    m_autoState = 0;
}

void Auton::Periodic() {
    if (m_autoSelected.compare("CalibrateWheels")) {
        CalibrateWheelsAuto();
    } else {
        
    }
}

units::time::second_t Auton::GetStateTime() {
    return frc::Timer::GetFPGATimestamp() - m_stateStartTime;
}

void Auton::NextState() {
    // Do NOT reset position or anything!! NextState should just reset time and increment state
    m_autoState++;
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
}