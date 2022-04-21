package frc.robot.commands.autonomous.middle_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MiddleFiveBallAuton extends SequentialCommandGroup {
    public MiddleFiveBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new MiddleThreeBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_MIDDLE.NAME, Poses.BALL_HUMAN_PLAYER.NAME),
            // Intake
            // Wait (for 2nd ball)
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_HUMAN_PLAYER.NAME, Poses.SHOOTING_SPOT_RIGHT.NAME)
            // Shoot
        );
    }
}
