package frc.robot.commands.autonomous.right_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightFiveBallAuton extends SequentialCommandGroup {
    public RightFiveBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new RightThreeBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_MIDDLE.NAME, Poses.BALL_HUMAN_PLAYER.NAME),
            // Intake
            // Wait (for 2nd ball)
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_HUMAN_PLAYER.NAME, Poses.SHOOTING_SPOT_LEFT.NAME)
            // Shoot
        );
    }
}
