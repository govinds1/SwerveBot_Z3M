package frc.robot.commands.autonomous.right_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightTwoBallAuton extends SequentialCommandGroup {
    public RightTwoBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new FollowTrajectoryCommand(swerveDrive, Poses.AUTON_RIGHT_START.NAME, Poses.BALL_RIGHT.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_RIGHT.NAME, Poses.SHOOTING_SPOT_RIGHT.NAME)
            // Shoot
        );
    }
}
