#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "SwerveDrive.h"
#include "PathManager.h"

class TryHardAuton {
    enum StartPosition {
        kLeft, kMiddle, kRight, kAbort
    };

    units::time::second_t m_stateStartTime = 0_s;
    std::shared_ptr<SwerveDrive> m_swerveDrive;
    std::shared_ptr<PathManager> m_pathManager;

    std::string m_autoSelected;
    int m_autoState = 0;

    int m_ballsDesired = 0;
    StartPosition m_startSide;
    int m_ballsShot = 0;
    frc::Pose2d m_startPose;
    int m_ballsHolding = 0;
    std::string m_currentPosition;
    std::string m_nextPosition;

    public:
    TryHardAuton(std::shared_ptr<SwerveDrive> swerveDrive, std::shared_ptr<PathManager> pathManager);
    void Init();
    void Periodic();
    void ShootBalls();
    void GrabNextBall();
    void HoldForTeleop();
    void SetPositions();

    units::time::second_t GetStateTime();
    void NextState();
};