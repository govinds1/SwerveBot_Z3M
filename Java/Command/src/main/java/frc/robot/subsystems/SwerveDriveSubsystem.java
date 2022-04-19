// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class SwerveDriveSubsystem extends SubsystemBase {
  WheelModuleSubsystem leftFront = new WheelModuleSubsystem(RobotMap.LEFT_FRONT_IDS, RobotMap.LEFT_FRONT_ENCODER_IDS);
  WheelModuleSubsystem leftRear = new WheelModuleSubsystem(RobotMap.LEFT_REAR_IDS, RobotMap.LEFT_REAR_ENCODER_IDS);
  WheelModuleSubsystem rightFront = new WheelModuleSubsystem(RobotMap.RIGHT_FRONT_IDS, RobotMap.RIGHT_FRONT_ENCODER_IDS);
  WheelModuleSubsystem rightRear = new WheelModuleSubsystem(RobotMap.RIGHT_REAR_IDS, RobotMap.RIGHT_REAR_ENCODER_IDS);

  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
