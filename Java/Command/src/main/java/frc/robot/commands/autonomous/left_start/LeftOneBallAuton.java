package frc.robot.commands.autonomous.left_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LeftOneBallAuton extends SequentialCommandGroup {
    public LeftOneBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new FollowTrajectoryCommand(swerveDrive, Poses.AUTON_LEFT_START.NAME, Poses.SHOOTING_SPOT_LEFT.NAME)
            // Shoot
        );
    }
}
