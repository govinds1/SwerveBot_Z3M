package frc.robot.commands.autonomous.middle_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MiddleThreeBallAuton extends SequentialCommandGroup {
    public MiddleThreeBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new MiddleTwoBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_MIDDLE.NAME, Poses.BALL_LEFT.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_LEFT.NAME, Poses.SHOOTING_SPOT_MIDDLE.NAME)
            // Shoot
        );
    }
}
