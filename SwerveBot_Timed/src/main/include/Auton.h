#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "SwerveDrive.h"

class Auton {
    units::time::second_t m_stateStartTime = 0_s;
    std::shared_ptr<SwerveDrive> m_swerveDrive;

    std::vector<std::string> m_autoList = {};
    std::string m_autoSelected;
    int m_autoState = 0;

    public:
    Auton(std::shared_ptr<SwerveDrive> swerveDrive);
    void Init();
    void Periodic();


    void CalibrateWheelsAuto();

    units::time::second_t GetStateTime();
    void NextState();
};