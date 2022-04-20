package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveCircleCommand extends CommandBase {
    private final SwerveDriveSubsystem m_swerveDrive;
    private double m_startTime;
    
    public DriveCircleCommand(SwerveDriveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

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
        if (GetTime() <= 3) {
            m_swerveDrive.Drive(0.3, 0, 0.3);
        } else {
            m_swerveDrive.Drive(
                Math.sin(GetTime() * 3.14 / 2.0) * 0.4,
                Math.cos(GetTime() * 3.14 / 2.0) * 0.4,
                Math.cos(GetTime() * 3.14 / 2.0) * 0.4
            );
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_swerveDrive.Drive(0.01, 0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return GetTime() >= 15.0;
    }

    public double GetTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }
}
