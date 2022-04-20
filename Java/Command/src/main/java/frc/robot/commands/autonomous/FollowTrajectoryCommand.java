package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathManager;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
    private final SwerveDriveSubsystem m_swerveDrive;
    private String m_trajectoryName;
    private double m_startTime;

    public FollowTrajectoryCommand(SwerveDriveSubsystem swerveDrive, String trajectoryName) {
        m_swerveDrive = swerveDrive;
        m_trajectoryName = trajectoryName;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    public FollowTrajectoryCommand(SwerveDriveSubsystem swerveDrive, String startName, String endName) {
        m_swerveDrive = swerveDrive;
        m_trajectoryName = PathManager.ConcatPoseNames(startName, endName);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
      m_startTime = Timer.getFPGATimestamp();
      m_swerveDrive.FollowTrajectory(GetTime(), m_trajectoryName);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_swerveDrive.FollowTrajectory(GetTime(), m_trajectoryName);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_swerveDrive.Drive(0.01, 0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Use time or atTarget?
      return PathManager.TrajectoryStatus(GetTime(), m_trajectoryName) >= 1.0;
    }

    public double GetTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }
}
