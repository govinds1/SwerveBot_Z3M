#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "SwerveDrive.h"
#include "PathManager.h"

class Auton {
    enum StartPosition {
        kLeft, kMiddle, kRight, kAbort
    };

    units::time::second_t m_stateStartTime = 0_s;
    std::shared_ptr<SwerveDrive> m_swerveDrive;
    std::shared_ptr<PathManager> m_pathManager;
    frc::Pose2d m_startPose;
    int m_ballsDesired;
    int m_ballsShot;
    int m_ballsHolding;
    StartPosition m_startSide;
    std::string m_currentPosition;

    std::string m_autoSelected;
    int m_autoState = 0;

    public:
    Auton(std::shared_ptr<SwerveDrive> swerveDrive, std::shared_ptr<PathManager> pathManager);
    void Init();
    void Periodic();
    void LeftStartAuto();
    void MiddleStartAuto();
    void RightStartAuto();
    void GoToPose(const std::string &poseName, bool shoot, bool intake);
    void WaitForMechanism(bool shoot, bool intake);

    units::time::second_t GetStateTime();
    double GetStateTimeValue();
    void NextState();
};