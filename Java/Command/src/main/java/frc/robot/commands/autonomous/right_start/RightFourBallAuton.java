package frc.robot.commands.autonomous.right_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightFourBallAuton extends SequentialCommandGroup {
    public RightFourBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new RightThreeBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_MIDDLE.NAME, Poses.BALL_LEFT.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_LEFT.NAME, Poses.SHOOTING_SPOT_LEFT.NAME)
            // Shoot
        );
    }
}
