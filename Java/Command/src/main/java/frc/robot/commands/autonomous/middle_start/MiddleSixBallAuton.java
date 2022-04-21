package frc.robot.commands.autonomous.middle_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MiddleSixBallAuton extends SequentialCommandGroup {
    public MiddleSixBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new MiddleFiveBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_RIGHT.NAME, Poses.BALL_RIGHT.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_RIGHT.NAME, Poses.SHOOTING_SPOT_RIGHT.NAME)
            // Shoot
        );
    }
}
