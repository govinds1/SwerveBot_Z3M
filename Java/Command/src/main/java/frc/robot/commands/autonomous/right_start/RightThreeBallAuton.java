package frc.robot.commands.autonomous.right_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightThreeBallAuton extends SequentialCommandGroup {
    public RightThreeBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new RightTwoBallAuton(swerveDrive),
            new FollowTrajectoryCommand(swerveDrive, Poses.SHOOTING_SPOT_RIGHT.NAME, Poses.BALL_MIDDLE.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_MIDDLE.NAME, Poses.SHOOTING_SPOT_MIDDLE.NAME)
            // Shoot
        );
    }
}
