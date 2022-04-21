// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveControlCommand;
import frc.robot.commands.autonomous.DriveCircleCommand;
import frc.robot.commands.autonomous.left_start.LeftFiveBallAuton;
import frc.robot.commands.autonomous.left_start.LeftFourBallAuton;
import frc.robot.commands.autonomous.left_start.LeftOneBallAuton;
import frc.robot.commands.autonomous.left_start.LeftSixBallAuton;
import frc.robot.commands.autonomous.left_start.LeftThreeBallAuton;
import frc.robot.commands.autonomous.left_start.LeftTwoBallAuton;
import frc.robot.commands.autonomous.right_start.RightFiveBallAuton;
import frc.robot.commands.autonomous.right_start.RightFourBallAuton;
import frc.robot.commands.autonomous.right_start.RightOneBallAuton;
import frc.robot.commands.autonomous.right_start.RightSixBallAuton;
import frc.robot.commands.autonomous.right_start.RightThreeBallAuton;
import frc.robot.commands.autonomous.right_start.RightTwoBallAuton;
import frc.robot.commands.autonomous.middle_start.MiddleFiveBallAuton;
import frc.robot.commands.autonomous.middle_start.MiddleFourBallAuton;
import frc.robot.commands.autonomous.middle_start.MiddleOneBallAuton;
import frc.robot.commands.autonomous.middle_start.MiddleSixBallAuton;
import frc.robot.commands.autonomous.middle_start.MiddleThreeBallAuton;
import frc.robot.commands.autonomous.middle_start.MiddleTwoBallAuton;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem m_swerveDrive = new SwerveDriveSubsystem();
  private final XboxController m_driveController = new XboxController(RobotMap.DRIVE_CONTROLLER_CHANNEL);

  private final DriveControlCommand m_driveControlCommand = new DriveControlCommand(m_swerveDrive, m_driveController);

  // Naming convention (no spaces): X-Y
  // X = starting position -> {Left, Middle, Right}
  // Y = # of balls -> {1, 2, 3, 4, 5, ?}
  private final String[] m_autoList = new String[19]; // common robot crash bug -> array overflow because this size isn't updated

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_swerveDrive.setDefaultCommand(m_driveControlCommand);
    m_autoList[0] = "Circle";
    for (int i = 1; (i+2) < m_autoList.length; i = i+3) {
      m_autoList[i] = "Left-"+ i+1;
      m_autoList[i+1] = "Middle-"+ i+1;
      m_autoList[i+2] = "Right-"+ i+1;
    }
    // Or use a custom dashboard widget -> select start and then select # of balls
    SmartDashboard.putStringArray("Auto List", m_autoList);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    Command defaultCommand = new DriveCircleCommand(m_swerveDrive);
    Command autoCommand = defaultCommand;
    String selected = SmartDashboard.getString("Auto Selector", "Circle");
    int numBalls = Integer.parseInt(selected.substring(selected.indexOf("-") + 1));
    if (selected.startsWith("L")) {
      m_swerveDrive.init(true, Poses.AUTON_LEFT_START.toPoseFeet());
      switch (numBalls) {
        case 1:
          autoCommand = new LeftOneBallAuton(m_swerveDrive);
          break;
        case 2:
          autoCommand = new LeftTwoBallAuton(m_swerveDrive);
          break;
        case 3:
          autoCommand = new LeftThreeBallAuton(m_swerveDrive);
          break;
        case 4:
          autoCommand = new LeftFourBallAuton(m_swerveDrive);
          break;
        case 5:
          autoCommand = new LeftFiveBallAuton(m_swerveDrive);
          break;
        case 6:
          autoCommand = new LeftSixBallAuton(m_swerveDrive);
          break;
        default:
          autoCommand = defaultCommand;
          break;
      }
    } else if (selected.startsWith("M")) {
      m_swerveDrive.init(true, Poses.AUTON_MIDDLE_START.toPoseFeet());
      switch (numBalls) {
        case 1:
          autoCommand = new MiddleOneBallAuton(m_swerveDrive);
          break;
        case 2:
          autoCommand = new MiddleTwoBallAuton(m_swerveDrive);
          break;
        case 3:
          autoCommand = new MiddleThreeBallAuton(m_swerveDrive);
          break;
        case 4:
          autoCommand = new MiddleFourBallAuton(m_swerveDrive);
          break;
        case 5:
          autoCommand = new MiddleFiveBallAuton(m_swerveDrive);
          break;
        case 6:
          autoCommand = new MiddleSixBallAuton(m_swerveDrive);
          break;
        default:
          autoCommand = defaultCommand;
          break;
      }
    } else if (selected.startsWith("R")) {
      m_swerveDrive.init(true, Poses.AUTON_RIGHT_START.toPoseFeet());
      switch (numBalls) {
        case 1:
          autoCommand = new RightOneBallAuton(m_swerveDrive);
          break;
        case 2:
          autoCommand = new RightTwoBallAuton(m_swerveDrive);
          break;
        case 3:
          autoCommand = new RightThreeBallAuton(m_swerveDrive);
          break;
        case 4:
          autoCommand = new RightFourBallAuton(m_swerveDrive);
          break;
        case 5:
          autoCommand = new RightFiveBallAuton(m_swerveDrive);
          break;
        case 6:
          autoCommand = new RightSixBallAuton(m_swerveDrive);
          break;
        default:
          autoCommand = defaultCommand;
          break;
      }
    } else {
      m_swerveDrive.init(false, Poses.AUTON_LEFT_START.toPoseFeet());
      autoCommand = defaultCommand;
    }

    return autoCommand;
  }
}
