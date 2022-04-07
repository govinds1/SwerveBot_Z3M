// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include "Turret.h"
#include "Teleop.h"
#include "Auton.h"
#include "PathManager.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:

  std::shared_ptr<PathManager> m_pathManager = std::make_shared<PathManager>();
  std::shared_ptr<SwerveDrive> m_drive = std::make_shared<SwerveDrive>(m_pathManager);
  std::shared_ptr<Turret> m_turret = std::make_shared<Turret>(m_drive);
  std::shared_ptr<Teleop> m_teleop = std::make_shared<Teleop>(m_drive, m_pathManager, m_turret);
  std::shared_ptr<Auton> m_auton = std::make_shared<Auton>(m_drive, m_pathManager, m_turret);
};
