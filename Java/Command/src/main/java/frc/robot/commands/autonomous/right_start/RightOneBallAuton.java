package frc.robot.commands.autonomous.right_start;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Poses;
import frc.robot.commands.autonomous.FollowTrajectoryCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightOneBallAuton extends SequentialCommandGroup {
    public RightOneBallAuton(SwerveDriveSubsystem swerveDrive) {
        addCommands(
            new FollowTrajectoryCommand(swerveDrive, Poses.AUTON_RIGHT_START.NAME, Poses.SHOOTING_SPOT_RIGHT.NAME)
            // Shoot
        );
    }
}
