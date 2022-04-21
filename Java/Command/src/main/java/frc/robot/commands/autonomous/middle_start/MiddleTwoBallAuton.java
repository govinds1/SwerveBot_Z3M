package frc.robot.commands.autonomous.middle_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MiddleTwoBallAuton extends SequentialCommandGroup {
    public MiddleTwoBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new FollowTrajectoryCommand(swerveDrive, Poses.AUTON_MIDDLE_START.NAME, Poses.BALL_MIDDLE.NAME),
            // Intake
            new FollowTrajectoryCommand(swerveDrive, Poses.BALL_MIDDLE.NAME, Poses.SHOOTING_SPOT_MIDDLE.NAME)
            // Shoot
        );
    }
}
