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
        m_currentPosition = POSES::AUTON_LEFT_START.NAME;
        break;
    case StartPosition::kMiddle:
        m_startPose = POSES::AUTON_MIDDLE_START.toPose();
        m_currentPosition = POSES::AUTON_MIDDLE_START.NAME;
        break;
    case StartPosition::kRight:
        m_startPose = POSES::AUTON_RIGHT_START.toPose();
        m_currentPosition = POSES::AUTON_RIGHT_START.NAME;
        break;
    default: // kAbort -> bad name
        setPose = false;
        break;
    }
    m_swerveDrive->Init(setPose, m_startPose);
    m_stateStartTime = frc::Timer::GetFPGATimestamp();
    m_autoState = 0;
    m_ballsShot = 0;
    m_pathManager->SetEnabled(true);
}

void Auton::Periodic() {
    switch (m_startSide) {
    case StartPosition::kLeft:
        LeftStartAuto();
        break;
    case StartPosition::kMiddle:
        MiddleStartAuto();
        break;
    case StartPosition::kRight:
        RightStartAuto();
        break;
    default: // kAbort -> bad name
        
        break;
    }
}

void Auton::LeftStartAuto() {

}

void Auton::MiddleStartAuto() {

}

void Auton::RightStartAuto() {
    if (m_ballsShot >= m_ballsDesired) {
        m_swerveDrive->FollowTrajectory(GetStateTime(), m_currentPosition, POSES::BALL_RIGHT.NAME);
        if (m_pathManager->AtTarget()) {
            m_pathManager->SetEnabled(false);
        }
    } else {
        switch (m_autoState) {
        case 0: // Get to first ball and run up shooter and intake
            GoToPose(POSES::BALL_RIGHT.NAME, true, true);
            break;
        case 1: // Intake first ball and run shooter to shoot pre-loaded and it
            WaitForMechanism(true, true);
            break;
        case 2: // Get to second ball
            GoToPose(POSES::BALL_MIDDLE.NAME, true, true);
            break;
        case 3: // Intake second ball and shoot it
            WaitForMechanism(true, true);
            break;
        case 4: // Get to third ball
            GoToPose(POSES::BALL_HUMAN_PLAYER.NAME, false, true);
            break;
        case 5: // Wait to intake third ball and fourth passed from human player
            WaitForMechanism(false, true);
            break;
        case 6: // Get to shooting spot
            GoToPose(POSES::SHOOTING_SPOT_RIGHT.NAME, true, false);
            break;
        case 7: // Shoot both
            WaitForMechanism(true, false);
            break;
        }
    }
}

void Auton::GoToPose(const std::string &poseName, bool shoot, bool intake) {
    double percentProgress = m_swerveDrive->FollowTrajectory(GetStateTime(), m_currentPosition, poseName);
    if (percentProgress >= 0.85) {
        if (shoot) {
            // run up shooter
        }
        if (intake) {
            // run intake
        }
    } else if (percentProgress >= 1.0 || m_pathManager->AtTarget()) {
        m_currentPosition = poseName;
        if (intake) {
            m_ballsHolding++;
        }
        m_swerveDrive->Drive(0, 0, 0);
        NextState();
    }
}

void Auton::WaitForMechanism(bool shoot, bool intake) {
    if (GetStateTime() >= (((shoot) ? AUTON::SHOOT_TIME : 0_s) + ((intake) ? AUTON::INTAKE_TIME : 0_s))) {
        if (intake) {
            m_ballsHolding++;
        }
        if (shoot) {
            m_ballsShot += m_ballsHolding;
            m_ballsHolding = 0;
        }
        NextState();
    } else {
        m_swerveDrive->Drive(0, 0, 0);
        if (shoot) {
            // keep shooter running
            // run conveyor
        }
        if (intake) {
            // keep intake running
            // drive forward a little, very slowly, to try and help pick up the ball
            m_swerveDrive->Drive(0.15, 0, 0);
        }
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