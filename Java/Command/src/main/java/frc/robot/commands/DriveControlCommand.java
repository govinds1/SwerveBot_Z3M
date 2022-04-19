// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Calibrations;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveControlCommand extends CommandBase {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final XboxController m_driveController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveControlCommand(SwerveDriveSubsystem swerveDrive, XboxController driveController) {
    m_swerveDrive = swerveDrive;
    m_driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.init(false, new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Possible control schemes:
    //  1. 2910 does it this way
    //    LeftY -> forward (x axis)
    //    LeftX -> strafe (y axis)
    //    RightX -> rotation (turn about z axis)
    //  2.
    //    LeftY -> forward (x axis)
    //    RightX -> strafe (y axis)
    //    LeftTrigger -> rotate left
    //    RightTrigger -> rotate right
    //  3. Two 3d flight sticks
    //    LeftY -> forward (x axis)
    //    LeftX -> strafe (y axis)
    //    RightZ -> rotation (turn about z axis)
    

    // Scheme 1
    double forward = m_driveController.getLeftY();
    double strafe = m_driveController.getLeftX();
    double rot = m_driveController.getRightX();

    // Scheme 2
    // double forward = m_driveController.getLeftY();
    // double strafe = m_driveController.getRightX();
    // double rot = m_driveController.getRightTriggerAxis() - m_driveController.getLeftTriggerAxis();

    
    forward = Utils.Deadband(forward, Calibrations.JOYSTICK_DEADZONE);
    strafe = Utils.Deadband(strafe, Calibrations.JOYSTICK_DEADZONE);
    rot = Utils.Deadband(rot, Calibrations.JOYSTICK_DEADZONE);
    
    forward = Utils.PowerKeepSign(forward, 2);
    strafe = Utils.PowerKeepSign(strafe, 2);
    rot = Utils.PowerKeepSign(rot, 2);
    
    if (m_driveController.getXButtonPressed()) {
      m_swerveDrive.SetFieldRelative(true);
    } else if (m_driveController.getBButtonPressed()) {
      m_swerveDrive.SetFieldRelative(false);
    }

    if (forward == 0 && strafe == 0 && rot == 0) {
      // Idle state
      // Replace with X formation function
      m_swerveDrive.Drive(0.01, 0, 0, false);
    } else {
      // Normal state
      m_swerveDrive.Drive(forward, strafe, rot);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.Drive(0.01, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
