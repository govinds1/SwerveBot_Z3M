#include "TryHardAuton.h"

TryHardAuton::TryHardAuton(std::shared_ptr<SwerveDrive> swerveDrive, std::shared_ptr<PathManager> pathManager) {
    m_swerveDrive = swerveDrive;
    m_pathManager = pathManager;
    frc::SmartDashboard::PutStringArray("Auto List", AUTON::AUTO_LIST);
    m_stateStartTime = 0_s;
}

void TryHardAuton::Init() {
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
        m_ballsHolding = 1; // pre-loaded
        m_nextPosition = "Ball 4";
        break;
    case StartPosition::kMiddle:
        m_startPose = POSES::AUTON_MIDDLE_START.toPose();
        m_currentPosition = "Middle Start";
        m_ballsHolding = 1; // pre-loaded
        m_nextPosition = "Ball 4";
        break;
    case StartPosition::kRight:
        m_startPose = POSES::AUTON_RIGHT_START.toPose();
        m_currentPosition = "Right Start";
        m_ballsHolding = 1;
        m_nextPosition = "Ball 1";
        break;
    default: // kAbort -> bad name
        setPose = false;
        break;
    }
    m_swerveDrive->Init(setPose, m_startPose);
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
    m_autoState = 0;
    m_ballsShot = 0;
}

void TryHardAuton::Periodic() {
    if (m_ballsShot >= m_ballsDesired) {
        HoldForTeleop();
    } else if (m_ballsHolding >= 2 || (m_ballsHolding >= (m_ballsDesired - m_ballsShot))) {
        ShootBalls();
    } else {
        GrabNextBall();
    }
}

void TryHardAuton::ShootBalls() {
    // Go to shooting location and shoot
    double percentProgress = m_swerveDrive->FollowTrajectory(GetStateTime(), m_currentPosition, m_nextPosition);
    if (percentProgress >= 0.7) {
        // run up shooter, don't fire yet
    } else if (percentProgress >= 1.2) {
        m_ballsShot += m_ballsHolding;
        m_ballsHolding = 0;
        SetPositions();
        NextState();
    }

}

void TryHardAuton::HoldForTeleop() {
    // Go to where you want to start teleop and wait
}

void TryHardAuton::GrabNextBall() {
    // Follow trajectory to next ball and run intake when you get close
    // Make sure you're facing the correct way
    double percentProgress = m_swerveDrive->FollowTrajectory(GetStateTime(), m_currentPosition, m_nextPosition);
    if (percentProgress >= 0.85) {
        // run intake
    } else if (percentProgress >= 1.15) {
        m_ballsHolding++;
        SetPositions();
        NextState();
    }
}

// If ballsHolding < 2 then set next to the next ball
// else set next to shoot
void TryHardAuton::SetPositions() {
    m_currentPosition = m_nextPosition;
    if (m_ballsHolding < 2) {
        m_nextPosition = "Ball " + std::to_string(m_ballsShot + m_ballsHolding + 1);
    } else {
        // Use start side to determine shoot side (good?)
        switch (m_startSide) {
        case StartPosition::kLeft:
            m_nextPosition = "Shoot Left";
            break;
        case StartPosition::kMiddle:
            m_nextPosition = "Shoot Middle";
            break;
        case StartPosition::kRight:
            m_nextPosition = "Shoot Right";
            break;
        default: // kAbort -> bad name
            m_nextPosition = "Shoot Right";
            break;
        }
    }
}

units::time::second_t TryHardAuton::GetStateTime() {
    return frc::Timer::GetFPGATimestamp() - m_stateStartTime;
}

void TryHardAuton::NextState() {
    // Do NOT reset position or anything!! NextState should just reset time and increment state
    m_autoState++;
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
}