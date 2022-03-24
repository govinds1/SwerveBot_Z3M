#include "Auton.h"

Auton::Auton(std::shared_ptr<SwerveDrive> swerveDrive, std::shared_ptr<PathManager> pathManager) {
    m_swerveDrive = swerveDrive;
    m_pathManager = pathManager;
    frc::SmartDashboard::PutStringArray("Auto List", AUTON::AUTO_LIST);
    m_stateStartTime = 0_s;
}

void Auton::Init() {
    m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", AUTON::AUTO_LIST[0]);
    int dashIdx = m_autoSelected.find('-');
    m_ballsDesired = std::stoi(m_autoSelected.substr(dashIdx + 1));
    if (std::tolower(m_autoSelected[0]) == 'l') {
        m_startSide = StartPosition::kLeft;
    } else if (std::tolower(m_autoSelected[0]) == 'm') {
        m_startSide = StartPosition::kMiddle;
    } else if (std::tolower(m_autoSelected[0]) == 'r') {
        m_startSide = StartPosition::kRight;
    } else {
        m_startSide = StartPosition::kAbort;
    }
    bool setPose = true;
    switch (m_startSide) {
        case StartPosition::kLeft:
            m_startPose = POSES::AUTON_LEFT_START.toPose();
            m_currentPosition = "Left Start";
            break;
        case StartPosition::kMiddle:
            m_startPose = POSES::AUTON_MIDDLE_START.toPose();
            m_currentPosition = "Middle Start";
            break;
        case StartPosition::kRight:
            m_startPose = POSES::AUTON_RIGHT_START.toPose();
            m_currentPosition = "Right Start";
            break;
        default: // kAbort -> bad name
            setPose = false;
            break;
    }
    m_swerveDrive->Init(setPose, m_startPose);
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
    m_autoState = 0;
    m_ballsShot = 0;
    m_ballsHolding = 0;
}

void Auton::Periodic() {
    if (m_ballsShot >= m_ballsDesired) {
        HoldForTeleop();
    } else if (m_ballsHolding >= 2 || (m_ballsHolding >= (m_ballsDesired - m_ballsShot))) {
        ShootBalls();
    } else {
        GrabNextBall();
    }
}

void Auton::ShootBalls() {
    // Go to shooting location and shoot
    m_ballsShot += m_ballsHolding;
    m_ballsHolding = 0;
}

void Auton::HoldForTeleop() {
    // Go to where you want to start teleop and wait
}

void Auton::GrabNextBall() {
    // Follow trajectory to next ball and run intake when you get close
    // Make sure you're facing the correct way
    
    
}

units::time::second_t Auton::GetStateTime() {
    return frc::Timer::GetFPGATimestamp() - m_stateStartTime;
}

void Auton::NextState() {
    // Do NOT reset position or anything!! NextState should just reset time and increment state
    m_autoState++;
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
}