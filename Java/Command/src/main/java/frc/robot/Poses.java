package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Poses {
// For correcting/setting odometry to certain known locations when possible
// Examples: Auton starts, scoring locations, corners of the field other landmarks, etc
// as a side note, it is possible to do a more dynamic calibration with vision, so adding in vision target locations is helpful as well
// Mainly used for spline trajectories, as start/end locations or waypoints
// Important for auton
// https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022LayoutMarkingDiagram.pdf
    public static class field_pose {
        double XFeet; // across length of field, postive towards forward (to opponent's alliance station), feet
        double YFeet; // across width of field, positive towards left, feet
        double ROTDegrees; // where 0 is facing the opponent's alliance station, positive towards turning left (CCW), degrees
        String NAME;

        Translation2d toTranslationFeet() {
            return new Translation2d(XFeet, YFeet);
        }
        Translation2d toTranslationMeters() {
            return Utils.TranslationFeetToMeters(toTranslationFeet());
        }
        Pose2d toPoseFeet() {
            return new Pose2d(toTranslationFeet(), Rotation2d.fromDegrees(ROTDegrees));
        }
        Pose2d toPoseMeters() {
            return Utils.PoseFeetToMeters(toPoseFeet());
        }
        field_pose(double x, double y, double rot, String name) {
            XFeet = x;
            YFeet = y;
            ROTDegrees = rot;
            NAME = name;
        }
    };
    // (0, 0, 0) means robot is in the center of the field, facing the opponent's alliance station
    // ^^^^^^^^^ or we change the world coordinates to whatever we want ^^^^^^^^^

    public static final field_pose AUTON_LEFT_START  = new field_pose( // flush against the hub, center of side
        -(Math.sin(24 * Math.PI / 180.0) * (67.81 + Calibrations.CHASSIS_LENGTH) / 12.0), // might be 21??
        -(Math.cos(24 * Math.PI / 180.0) * (67.81 + Calibrations.CHASSIS_LENGTH) / 12.0),
        24,
        "Start_L"
    ); 
    public static final field_pose AUTON_MIDDLE_START  = new field_pose( // on edge of tarmac, facing hub and opponent's alliance station
        -((Math.cos(24 * Math.PI / 180.0) * (-(118.66 / 12.0))) - Calibrations.CHASSIS_LENGTH),
        0,
        0,
        "Start_M"
    );
    public static final field_pose AUTON_RIGHT_START  = new field_pose( // flush against the hub, center of side
        -(Math.cos(24 * Math.PI / 180.0) * (67.81 + Calibrations.CHASSIS_LENGTH) / 12.0),
        (Math.sin(24 * Math.PI / 180.0) * (67.81 + Calibrations.CHASSIS_LENGTH) / 12.0),
        -24,
        "Start_R"
    );

    // For try hard auton where you just "recursively" go through balls
    // public static final field_pose BALL_1 = new field_pose(0, 0, 0, "Ball 1"); // Far right ball
    // public static final field_pose BALL_2 = new field_pose(0, 0, 0, "Ball 2"); // Right side ball closer to center
    // public static final field_pose BALL_3 = new field_pose(0, 0, 0, "Ball 3"); // Human player station
    // public static final field_pose BALL_4 = new field_pose(0, 0, 0, "Ball 4"); // Far left ball

    // Set these Poses for exactly where the Ball is
    public static final field_pose BALL_RIGHT = new field_pose( // Ball on far right
        -35.5 / 12.0,
        -151 / 12.0,
        -90,
        "Ball_R"
    );
    public static final field_pose BALL_MIDDLE = new field_pose( // Ball in center-ish
        -127.742 / 12.0,
        -89.54 / 12.0,
        180.0,
        "Ball_M"
    );
    public static final field_pose BALL_LEFT = new field_pose( // Ball on left
        -127.742 / 12.0,
        89.54 / 12.0,
        180.0,
        "Ball_L"
    );
    public static final field_pose BALL_HUMAN_PLAYER = new field_pose( // Ball by human player station
        -285 / 12.0,
        -129.65 / 12.0,
        -136.25,
        "Ball_HP"
    );
    
    // Ball Waypoints, one for each ball spot
    // Used so that the robot comes at each ball from the right direction so the intake can actually pick it up
    // should be one or two feet away from the ball, so that it can drive forward while intaking
    public static final field_pose BALL_RIGHT_WAYPOINT = new field_pose( // Ball on far right
        BALL_RIGHT.XFeet,
        BALL_RIGHT.YFeet + (Calibrations.CHASSIS_LENGTH * 2 / 12.0),
        BALL_RIGHT.ROTDegrees,
        "WPT_Ball_R"
    );
    public static final field_pose BALL_MIDDLE_WAYPOINT = new field_pose( // Ball in center-ish
        BALL_MIDDLE.XFeet + (Calibrations.CHASSIS_LENGTH * 2 / 12.0),
        BALL_MIDDLE.YFeet,
        BALL_MIDDLE.ROTDegrees,
        "WPT_Ball_M"
    );
    public static final field_pose BALL_LEFT_WAYPOINT = new field_pose( // Ball on left
        BALL_LEFT.XFeet + (Calibrations.CHASSIS_LENGTH * 2 / 12.0),
        BALL_LEFT.YFeet,
        BALL_LEFT.ROTDegrees,
        "WPT_Ball_L"
    );
    public static final field_pose BALL_HUMAN_PLAYER_WAYPOINT = new field_pose( // Ball by human player station
        BALL_HUMAN_PLAYER.XFeet + (Calibrations.CHASSIS_LENGTH * 2 / 12.0), // not exact
        BALL_HUMAN_PLAYER.YFeet + (Calibrations.CHASSIS_WIDTH * 2 / 12.0), // not exact
        BALL_HUMAN_PLAYER.ROTDegrees,
        "WPT_Ball_HP"
    );

    // these might be the same as a ball pose
    // these should be exact where you want, face the hub but no need to come at it from a certain side
    public static final field_pose SHOOTING_SPOT_LEFT = new field_pose(
        -50 / 12.0,
        110 / 12.0,
        -30,
        "Shoot_L"
    );  
    public static final field_pose SHOOTING_SPOT_MIDDLE = new field_pose(
        -112 / 12.0,
        0,
        0,
        "Shoot_M"
    );
    public static final field_pose SHOOTING_SPOT_RIGHT = new field_pose(
        -50 / 12.0,
        -110 / 12.0,
        30,
        "Shoot_R"
    );
    public static final field_pose LAUNCH_PAD = new field_pose(  // P for (launch) pad
        (-195.25 + (Calibrations.CHASSIS_LENGTH / 2)) / 12.0,
        46 / 12.0,
        0,
        "Shoot_P"
    );
    public static final field_pose LAUNCH_PAD_WAYPOINT1 = new field_pose( // avoid coming at LP from hangar support
        (-195.25 + (Calibrations.CHASSIS_LENGTH * 2)) / 12.0,
        46 / 12.0,
        0,
        "WPT_Shoot_P_1"
    ); // use central_waypoint as well?
    // public static final field_pose LAUNCH_PAD_WAYPOINT2 = new field_pose((-195.25 / 12.0) + (Calibrations.CHASSIS_LENGTH * 2), 46 / 12.0, 0, "WPT_Shoot_P_2");


    public static final field_pose IMPORTANT_WAYPOINT_EXAMPLE = new field_pose(0, 0, 0, "Waypoint Example"); // could be a start or end position, but also a useful waypoint for a trajectory where toTranslation is used
    public static final field_pose CENTRAL_WAYPOINT = new field_pose( // Good place to put as a waypoint to not hit the hangar and for good lines to other places
        -200.0 / 12.0,
        -65.0 / 12.0,
        0,
        "WPT_Central"
    );


    public static final Pose2d trajectoryTolerance = new Pose2d(0.05, 0.05, new Rotation2d(0.05));
}
