package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathManager;
import frc.robot.subsystems.SwerveDriveSubsystem;


// For manually going to a specific Pose using the PID Controllers set up in PathManager
// Not following a pre-generated trajectory
// Will travel in a straight line and decelerate linearly
// Useful for teleop (????)
public class GoToPose extends CommandBase {
    private final SwerveDriveSubsystem m_swerveDrive;
    private Pose2d m_goalPose;
    private double m_startTime;

    public GoToPose(SwerveDriveSubsystem swerveDrive, Pose2d goalPose) {
        m_swerveDrive = swerveDrive;
        m_goalPose = goalPose;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
      m_startTime = Timer.getFPGATimestamp();
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = m_swerveDrive.GetPose();
        m_swerveDrive.SetDesiredSpeeds(PathManager.ManuallyCalculateSpeeds(currentPose, m_goalPose));
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_swerveDrive.Drive(0, 0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Use holonomic controller at reference or just check all 3 PID controllers
        return PathManager.AtTarget();
    }

    public double GetTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }
}
