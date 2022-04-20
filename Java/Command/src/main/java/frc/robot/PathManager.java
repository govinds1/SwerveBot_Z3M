package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;

public class PathManager {
    private static HashMap<String, Trajectory> trajectories;
    private static PIDController xController = new PIDController(Calibrations.CHASSIS_X[0], Calibrations.CHASSIS_X[1], Calibrations.CHASSIS_X[2]); // adds control for forward-backward error
    private static PIDController yController = new PIDController(Calibrations.CHASSIS_Y[0], Calibrations.CHASSIS_Y[1], Calibrations.CHASSIS_Y[2]); // adds control for left-right error
    private static ProfiledPIDController thetaController = new ProfiledPIDController( // adds control for azimuth error
        Calibrations.CHASSIS_ROT[0], Calibrations.CHASSIS_ROT[1], Calibrations.CHASSIS_ROT[2],
        new TrapezoidProfile.Constraints(Calibrations.MAX_CHASSIS_TURN_SPEED, Calibrations.MAX_CHASSIS_TURN_ACCEL)
    );
    private static HolonomicDriveController swerveDriveController = new HolonomicDriveController(xController, yController, thetaController);

    public PathManager() {
        // Add Trajectories here
        Poses.field_pose[] center_waypoint = {Poses.CENTRAL_WAYPOINT};
        GenTrajectory(Poses.AUTON_LEFT_START, Poses.BALL_RIGHT, center_waypoint);
        GenTrajectory(Poses.AUTON_MIDDLE_START, Poses.BALL_RIGHT);
        GenTrajectory(Poses.AUTON_RIGHT_START, Poses.BALL_RIGHT);

        GenTrajectory(Poses.AUTON_LEFT_START, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.AUTON_MIDDLE_START, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.AUTON_RIGHT_START, Poses.BALL_MIDDLE);

        GenTrajectory(Poses.AUTON_LEFT_START, Poses.BALL_LEFT);
        GenTrajectory(Poses.AUTON_MIDDLE_START, Poses.BALL_LEFT);
        GenTrajectory(Poses.AUTON_RIGHT_START, Poses.BALL_LEFT, center_waypoint);

        GenTrajectory(Poses.AUTON_LEFT_START, Poses.BALL_HUMAN_PLAYER, center_waypoint);
        GenTrajectory(Poses.AUTON_MIDDLE_START, Poses.BALL_HUMAN_PLAYER, center_waypoint);
        GenTrajectory(Poses.AUTON_RIGHT_START, Poses.BALL_HUMAN_PLAYER, center_waypoint);

        GenTrajectory(Poses.BALL_RIGHT, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.BALL_RIGHT, Poses.BALL_LEFT, center_waypoint);
        GenTrajectory(Poses.BALL_RIGHT, Poses.BALL_HUMAN_PLAYER, center_waypoint);
        GenTrajectory(Poses.BALL_RIGHT, Poses.SHOOTING_SPOT_LEFT, center_waypoint);
        GenTrajectory(Poses.BALL_RIGHT, Poses.SHOOTING_SPOT_MIDDLE, center_waypoint);
        GenTrajectory(Poses.BALL_RIGHT, Poses.SHOOTING_SPOT_RIGHT);

        GenTrajectory(Poses.BALL_MIDDLE, Poses.BALL_RIGHT);
        GenTrajectory(Poses.BALL_MIDDLE, Poses.BALL_LEFT);
        GenTrajectory(Poses.BALL_MIDDLE, Poses.BALL_HUMAN_PLAYER);
        GenTrajectory(Poses.BALL_MIDDLE, Poses.SHOOTING_SPOT_LEFT);
        GenTrajectory(Poses.BALL_MIDDLE, Poses.SHOOTING_SPOT_MIDDLE);
        GenTrajectory(Poses.BALL_MIDDLE, Poses.SHOOTING_SPOT_RIGHT);

        GenTrajectory(Poses.BALL_LEFT, Poses.BALL_RIGHT);
        GenTrajectory(Poses.BALL_LEFT, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.BALL_LEFT, Poses.BALL_HUMAN_PLAYER);
        GenTrajectory(Poses.BALL_LEFT, Poses.SHOOTING_SPOT_LEFT);
        GenTrajectory(Poses.BALL_LEFT, Poses.SHOOTING_SPOT_MIDDLE);
        GenTrajectory(Poses.BALL_LEFT, Poses.SHOOTING_SPOT_RIGHT);

        GenTrajectory(Poses.BALL_HUMAN_PLAYER, Poses.BALL_RIGHT, center_waypoint);
        GenTrajectory(Poses.BALL_HUMAN_PLAYER, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.BALL_HUMAN_PLAYER, Poses.BALL_LEFT, center_waypoint);
        GenTrajectory(Poses.BALL_HUMAN_PLAYER, Poses.SHOOTING_SPOT_LEFT, center_waypoint);
        GenTrajectory(Poses.BALL_HUMAN_PLAYER, Poses.SHOOTING_SPOT_MIDDLE, center_waypoint);
        GenTrajectory(Poses.BALL_HUMAN_PLAYER, Poses.SHOOTING_SPOT_RIGHT, center_waypoint);

        GenTrajectory(Poses.SHOOTING_SPOT_LEFT, Poses.BALL_RIGHT, center_waypoint);
        GenTrajectory(Poses.SHOOTING_SPOT_LEFT, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.SHOOTING_SPOT_LEFT, Poses.BALL_LEFT);
        GenTrajectory(Poses.SHOOTING_SPOT_LEFT, Poses.BALL_HUMAN_PLAYER, center_waypoint);

        GenTrajectory(Poses.SHOOTING_SPOT_MIDDLE, Poses.BALL_RIGHT, center_waypoint);
        GenTrajectory(Poses.SHOOTING_SPOT_MIDDLE, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.SHOOTING_SPOT_MIDDLE, Poses.BALL_LEFT);
        GenTrajectory(Poses.SHOOTING_SPOT_MIDDLE, Poses.BALL_HUMAN_PLAYER, center_waypoint);

        GenTrajectory(Poses.SHOOTING_SPOT_RIGHT, Poses.BALL_RIGHT);
        GenTrajectory(Poses.SHOOTING_SPOT_RIGHT, Poses.BALL_MIDDLE);
        GenTrajectory(Poses.SHOOTING_SPOT_RIGHT, Poses.BALL_LEFT, center_waypoint);
        GenTrajectory(Poses.SHOOTING_SPOT_RIGHT, Poses.BALL_HUMAN_PLAYER, center_waypoint);

        swerveDriveController.setTolerance(Poses.trajectoryTolerance);
    }

    public static String ConcatPoseNames(String pose1, String pose2) {
        return pose1 + " to " + pose2;
    }

    public static void AddTrajectory(String name, Trajectory trajectory) {
        trajectories.put(name, trajectory);
    }

    public static void AddTrajectory(String startPoseName, String endPoseName, Trajectory trajectory) {
        AddTrajectory(ConcatPoseNames(startPoseName, endPoseName), trajectory);
    }

    // All field_poses in Poses class are in feet, must be converted to meters!!
    // same with TrajectoryConfig constraints
    public static void GenTrajectory(Poses.field_pose start, Poses.field_pose end, Poses.field_pose[] waypoints, double max_vel, double max_accel) {
        List<Translation2d> wpts = new ArrayList<Translation2d>();
        for (Poses.field_pose wpt : waypoints) {
            wpts.add(wpt.toTranslationMeters());
        }
        if (end.NAME.startsWith("B")) {
            int underscoreIdx = end.NAME.lastIndexOf("_");
            if (end.NAME.startsWith("R", underscoreIdx + 1)) {
                wpts.add(Poses.BALL_RIGHT_WAYPOINT.toTranslationMeters());
            } else if (end.NAME.startsWith("M", underscoreIdx + 1)) {
                wpts.add(Poses.BALL_RIGHT_WAYPOINT.toTranslationMeters());
            } else if (end.NAME.startsWith("L", underscoreIdx + 1)) {
                wpts.add(Poses.BALL_RIGHT_WAYPOINT.toTranslationMeters());
            } else if (end.NAME.startsWith("H", underscoreIdx + 1)) {
                wpts.add(Poses.BALL_HUMAN_PLAYER_WAYPOINT.toTranslationMeters());
            } 
        }
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(max_vel), Units.feetToMeters(max_accel));
        // change config variables (startVel, endVel, reversed, constraints) if necessary


        AddTrajectory(start.NAME, end.NAME, TrajectoryGenerator.generateTrajectory(start.toPoseMeters(), wpts, end.toPoseMeters(), config));
    }

    public static void GenTrajectory(Poses.field_pose start, Poses.field_pose end, Poses.field_pose[] waypoints) {
        GenTrajectory(start, end, waypoints, Calibrations.MAX_CHASSIS_SPEED, Calibrations.MAX_CHASSIS_ACCEL);
    }

    public static void GenTrajectory(Poses.field_pose start, Poses.field_pose end) {
        Poses.field_pose[] empty = {};
        GenTrajectory(start, end, empty, Calibrations.MAX_CHASSIS_SPEED, Calibrations.MAX_CHASSIS_ACCEL);
    }

    public static ChassisSpeeds CalculateSpeeds(double currentTime, String trajectoryName, Pose2d currentPose, Rotation2d currentHeading) {
        State desiredState = trajectories.get(trajectoryName).sample(currentTime);
        return swerveDriveController.calculate(currentPose, desiredState, currentHeading);
    }

    public static ChassisSpeeds CalculateSpeeds(double currentTime, String startPoseName, String endPoseName, Pose2d currentPose, Rotation2d currentHeading) {
        return CalculateSpeeds(currentTime, ConcatPoseNames(startPoseName, endPoseName), currentPose, currentHeading);
    }

    public static double TrajectoryStatus(double currentTime, String trajectoryName) {
        return (currentTime / trajectories.get(trajectoryName).getTotalTimeSeconds());
    }

    public static boolean AtTarget() {
        return swerveDriveController.atReference();
    }

    public static void SetEnabled(boolean enabled) {
        swerveDriveController.setEnabled(enabled);
    }
}
