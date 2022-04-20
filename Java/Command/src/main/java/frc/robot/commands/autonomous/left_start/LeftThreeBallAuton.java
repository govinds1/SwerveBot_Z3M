package frc.robot.commands.autonomous.left_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LeftThreeBallAuton extends SequentialCommandGroup {
    public LeftThreeBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new LeftTwoBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_LEFT.NAME, Poses.BALL_MIDDLE.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_MIDDLE.NAME, Poses.SHOOTING_SPOT_MIDDLE.NAME)
            // Shoot
        );
    }
}
