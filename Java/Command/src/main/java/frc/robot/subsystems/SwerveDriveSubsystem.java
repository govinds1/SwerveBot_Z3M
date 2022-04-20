// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.PathManager;
import frc.robot.RobotMap;
import frc.robot.Utils;

public class SwerveDriveSubsystem extends SubsystemBase {
  WheelModuleSubsystem m_leftFront = new WheelModuleSubsystem(RobotMap.LEFT_FRONT_IDS, RobotMap.LEFT_FRONT_ENCODER_IDS, Calibrations.LEFT_FRONT_ZERO);
  WheelModuleSubsystem m_leftRear = new WheelModuleSubsystem(RobotMap.LEFT_REAR_IDS, RobotMap.LEFT_REAR_ENCODER_IDS, Calibrations.LEFT_REAR_ZERO);
  WheelModuleSubsystem m_rightFront = new WheelModuleSubsystem(RobotMap.RIGHT_FRONT_IDS, RobotMap.RIGHT_FRONT_ENCODER_IDS, Calibrations.RIGHT_FRONT_ZERO);
  WheelModuleSubsystem m_rightRear = new WheelModuleSubsystem(RobotMap.RIGHT_REAR_IDS, RobotMap.RIGHT_REAR_ENCODER_IDS, Calibrations.RIGHT_REAR_ZERO);

  Translation2d m_leftFrontLocation = new Translation2d(Calibrations.WHEELBASE_LENGTH / 2.0, Calibrations.WHEELBASE_WIDTH / 2.0);
  Translation2d m_leftRearLocation = new Translation2d(-Calibrations.WHEELBASE_LENGTH / 2.0, Calibrations.WHEELBASE_WIDTH / 2.0);
  Translation2d m_rightFrontLocation = new Translation2d(Calibrations.WHEELBASE_LENGTH / 2.0, -Calibrations.WHEELBASE_WIDTH / 2.0);
  Translation2d m_rightRearLocation = new Translation2d(-Calibrations.WHEELBASE_LENGTH / 2.0, -Calibrations.WHEELBASE_WIDTH / 2.0);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_leftFrontLocation, m_leftRearLocation, m_rightFrontLocation, m_rightRearLocation);
  ChassisSpeeds m_desiredSpeeds;
  ChassisSpeeds m_trueSpeeds;
  SwerveDriveOdometry m_odometry;

  ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  boolean fieldRelative = false;

  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem() {
    m_odometry = new SwerveDriveOdometry(m_kinematics, GetAngle());
    fieldRelative = false;
  }

  public void init(boolean autonomous, Pose2d initialPose) {
    // Use hand set Pose from selected auton to Set odometry here
    // If run in teleop init, or there is no selected auton, Pose should just be origin
    // do not reset pose in both teleop and auton, unless you know the exact poses
    if (autonomous) {
      SetPose(initialPose);
    }

    ResetSpeeds();
    m_leftFront.init();
    m_leftRear.init();
    m_rightFront.init();
    m_rightRear.init();
  }

  @Override
  public void periodic() {
    // Wheel States should update first
    m_leftFront.periodic();
    m_leftRear.periodic();
    m_rightFront.periodic();
    m_rightRear.periodic();

    m_odometry.updateWithTime(
        Timer.getFPGATimestamp(),
        GetAngle(), 
        m_leftFront.GetTrueState(),
        m_leftRear.GetTrueState(),
        m_rightFront.GetTrueState(),
        m_rightRear.GetTrueState()
    );

    SetTrueSpeeds();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Takes direct input from controller axes, handle conversions to real units and proper robot coordinates in this function
  // Robot coords: 
  //      forward = +x
  //      right = -y
  //      turn = -omega (CCW (turning left) is positive)
  public void Drive(double forward, double right, double turn) {
    SetDesiredSpeeds(
        forward * Calibrations.MAX_FORWARD_SPEED,
        -right * Calibrations.MAX_STRAFE_SPEED,
        -turn * Calibrations.MAX_TURN_SPEED
    );
  }

  public void Drive(double forward, double right, double turn, boolean driveFieldRelative) {
    boolean fieldRelSaved = fieldRelative;
    SetFieldRelative(driveFieldRelative);
    Drive(forward, right, turn);
    SetFieldRelative(fieldRelSaved);
  }

  public void FollowTrajectory(double currentTime, String trajectoryName) {
    SetDesiredSpeeds(PathManager.CalculateSpeeds(currentTime, trajectoryName, m_odometry.getPoseMeters(), GetAngle()));
  }

  public void FollowTrajectory(double currentTime, String startPoseName, String endPoseName) {
    FollowTrajectory(currentTime, PathManager.ConcatPoseNames(startPoseName, endPoseName));
  }

  public void ResetSpeeds() {
    SetDesiredSpeeds(0, 0, 0);
  }

  // Zeroing options:
  //  Manually -> Turn all wheels to face forward and then calibrate (store the absolute encoder position or set encoder position to 0)
  //  Temporary Limit Switch -> Have a pin/limit switch so the wheel can spin until it hits it and stops (facing forward), then calibrate
  //  https://www.chiefdelphi.com/t/swerve-zeroing/181799
  public void CalibrateWheelsManually() {
    // Should only be called when all wheels are facing front (at the "zero" position)
    // Sets the angle encoder zeros to the current reading and stores them

    m_leftFront.CalibrateAngle();
    m_leftRear.CalibrateAngle();
    m_rightFront.CalibrateAngle();
    m_rightRear.CalibrateAngle();

    Calibrations.LEFT_FRONT_ZERO = m_leftFront.GetRawAngle();
    Calibrations.LEFT_REAR_ZERO = m_leftRear.GetRawAngle();
    Calibrations.RIGHT_FRONT_ZERO = m_rightFront.GetRawAngle();
    Calibrations.RIGHT_REAR_ZERO = m_rightRear.GetRawAngle();
    // print these to dashboard or save to file on disabled init
  }

  // vx and vy in feet per second, must be converted
  // omega in radians per second
  public void SetDesiredSpeeds(double vxFeetPerSecond, double vyFeetPerSecond, double omega) {
    double vxMpS = Units.feetToMeters(vxFeetPerSecond);
    double vyMpS = Units.feetToMeters(vyFeetPerSecond);
    if (fieldRelative) {
        SetDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxMpS, vyMpS, omega, GetAngle()));
    } else {
        ChassisSpeeds newSpeeds = new ChassisSpeeds();
        newSpeeds.vxMetersPerSecond = vxMpS;
        newSpeeds.vyMetersPerSecond = vyMpS;
        newSpeeds.omegaRadiansPerSecond = omega;
        SetDesiredSpeeds(newSpeeds);
    }
  }

  public void SetDesiredSpeeds(ChassisSpeeds newSpeeds) {
    m_desiredSpeeds = newSpeeds;
    SetWheelStates();
  }

  public void SetTrueSpeeds() {
    m_trueSpeeds = m_kinematics.toChassisSpeeds(
      m_leftFront.GetTrueState(),
      m_leftRear.GetTrueState(),
      m_rightFront.GetTrueState(),
      m_rightRear.GetTrueState()
    );
  }

  public void SetWheelStates() {

    SwerveModuleState[] statesArray = m_kinematics.toSwerveModuleStates(m_desiredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(statesArray, Units.feetToMeters(Calibrations.MAX_FORWARD_SPEED + Calibrations.MAX_STRAFE_SPEED));
    m_leftFront.SetDesiredState(statesArray[0]);
    m_leftRear.SetDesiredState(statesArray[1]);
    m_rightFront.SetDesiredState(statesArray[2]);
    m_rightRear.SetDesiredState(statesArray[3]);
  }

  public void SetFieldRelative(boolean fieldRel) {
    fieldRelative = fieldRel;
  }

  public void SetPose(Pose2d newPose) {
      // newPose contains x, y, and rot
      // ResetPosition requires newPose and the current gyroAngle for an offset
      m_odometry.resetPosition(newPose, GetAngle());
  }

  public void SetPose(double x, double y) {
      Pose2d newPose = new Pose2d(x, y, GetAngle());
      SetPose(newPose);
  }

  public void ResetGyro() {
    m_gyro.reset();

    // SetPose calls m_odometry.resetPosition which uses the current (after resetting) gyro angle as the offset
    SetPose(GetPose());
  }

  public Rotation2d GetAngle() {
    return m_gyro.getRotation2d();
  }

  public Pose2d GetPose() {
    return Utils.PoseMetersToFeet(m_odometry.getPoseMeters());
  }

  public ChassisSpeeds GetDesiredSpeeds() {
    return m_desiredSpeeds;
  }

  public ChassisSpeeds GetTrueSpeeds() {
    return m_trueSpeeds;
  }
}
