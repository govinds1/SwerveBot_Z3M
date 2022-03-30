#pragma once

#include "Constants.h"


// For correcting/setting odometry to certain known locations when possible
// Examples: Auton starts, scoring locations, corners of the field other landmarks, etc
// as a side note, it is possible to do a more dynamic calibration with vision, so adding in vision target locations is helpful as well
// Mainly used for spline trajectories, as start/end locations or waypoints
// Important for auton
// https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022LayoutMarkingDiagram.pdf
namespace POSES {
    struct field_pose {
        double X; // across length of field, postive towards forward (to opponent's alliance station), feet
        double Y; // across width of field, positive towards left, feet
        double ROT; // where 0 is facing the opponent's alliance station, positive towards turning left (CCW), degrees
        std::string NAME;

        frc::Translation2d toTranslation() const {
            return frc::Translation2d(units::foot_t(X), units::foot_t(Y));
        }
        frc::Pose2d toPose() const {
            return frc::Pose2d(this->toTranslation(), frc::Rotation2d(units::angle::degree_t(ROT)));
        }
        field_pose(double x, double y, double rot, std::string name) {
            X = x;
            Y = y;
            ROT = rot;
            NAME = name;
        }
    };
    // (0, 0, 0) means robot is in the center of the field, facing the opponent's alliance station
    // ^^^^^^^^^ or we change the world coordinates to whatever we want ^^^^^^^^^

    const field_pose AUTON_LEFT_START { // flush against the hub, center of side
        -(std::sin(24 * M_PI / 180.0) * (67.81 + DIMENSIONS::CHASSIS_LENGTH.value()) / 12.0), // might be 21??
        -(std::cos(24 * M_PI / 180.0) * (67.81 + DIMENSIONS::CHASSIS_LENGTH.value()) / 12.0),
        24,
        "Start_L"
    }; 
    const field_pose AUTON_MIDDLE_START { // on edge of tarmac, facing hub and opponent's alliance station
        -((std::cos(24 * M_PI / 180.0) * (-(118.66 / 12.0))) - DIMENSIONS::CHASSIS_LENGTH.value()),
        0,
        0,
        "Start_M"
    };
    const field_pose AUTON_RIGHT_START { // flush against the hub, center of side
        -(std::cos(24 * M_PI / 180.0) * (67.81 + DIMENSIONS::CHASSIS_LENGTH.value()) / 12.0),
        (std::sin(24 * M_PI / 180.0) * (67.81 + DIMENSIONS::CHASSIS_LENGTH.value()) / 12.0),
        -24,
        "Start_R"
    };

    // For try hard auton where you just "recursively" go through balls
    // const field_pose BALL_1{0, 0, 0, "Ball 1"}; // Far right ball
    // const field_pose BALL_2{0, 0, 0, "Ball 2"}; // Right side ball closer to center
    // const field_pose BALL_3{0, 0, 0, "Ball 3"}; // Human player station
    // const field_pose BALL_4{0, 0, 0, "Ball 4"}; // Far left ball

    // Set these Poses for exactly where the Ball is
    const field_pose BALL_RIGHT{ // Ball on far right
        -35.5 / 12.0,
        -151 / 12.0,
        -90,
        "Ball_R"
    };
    const field_pose BALL_MIDDLE{ // Ball in center-ish
        -127.742 / 12.0,
        -89.54 / 12.0,
        180.0,
        "Ball_M"
    };
    const field_pose BALL_LEFT{ // Ball on left
        -127.742 / 12.0,
        89.54 / 12.0,
        180.0,
        "Ball_L"
    };
    const field_pose BALL_HUMAN_PLAYER{ // Ball by human player station
        -285 / 12.0,
        -129.65 / 12.0,
        -136.25,
        "Ball_HP"
    };
    
    // Ball Waypoints, one for each ball spot
    // Used so that the robot comes at each ball from the right direction so the intake can actually pick it up
    // should be one or two feet away from the ball, so that it can drive forward while intaking
    const field_pose BALL_RIGHT_WAYPOINT{ // Ball on far right
        BALL_RIGHT.X,
        BALL_RIGHT.Y + (DIMENSIONS::CHASSIS_LENGTH.value() * 2 / 12.0),
        BALL_RIGHT.ROT,
        "WPT_Ball_R"
    };
    const field_pose BALL_MIDDLE_WAYPOINT{ // Ball in center-ish
        BALL_MIDDLE.X + (DIMENSIONS::CHASSIS_LENGTH.value() * 2 / 12.0),
        BALL_MIDDLE.Y,
        BALL_MIDDLE.ROT,
        "WPT_Ball_M"
    };
    const field_pose BALL_LEFT_WAYPOINT{ // Ball on left
        BALL_LEFT.X + (DIMENSIONS::CHASSIS_LENGTH.value() * 2 / 12.0),
        BALL_LEFT.Y,
        BALL_LEFT.ROT,
        "WPT_Ball_L"
    };
    const field_pose BALL_HUMAN_PLAYER_WAYPOINT{ // Ball by human player station
        BALL_HUMAN_PLAYER.X + (DIMENSIONS::CHASSIS_LENGTH.value() * 2 / 12.0), // not exact
        BALL_HUMAN_PLAYER.Y + (DIMENSIONS::CHASSIS_WIDTH.value() * 2 / 12.0), // not exact
        BALL_HUMAN_PLAYER.ROT,
        "WPT_Ball_HP"
    };

    // these might be the same as a ball pose
    // these should be exact where you want, face the hub but no need to come at it from a certain side
    const field_pose SHOOTING_SPOT_LEFT{
        -50 / 12.0,
        110 / 12.0,
        -30,
        "Shoot_L"
    };  
    const field_pose SHOOTING_SPOT_MIDDLE{
        -112 / 12.0,
        0,
        0,
        "Shoot_M"
    };
    const field_pose SHOOTING_SPOT_RIGHT{
        -50 / 12.0,
        -110 / 12.0,
        30,
        "Shoot_R"
    };
    const field_pose LAUNCH_PAD{  // P for (launch) pad
        (-195.25 + (DIMENSIONS::CHASSIS_LENGTH.value() / 2)) / 12.0,
        46 / 12.0,
        0,
        "Shoot_P"
    };
    const field_pose LAUNCH_PAD_WAYPOINT1{ // avoid coming at LP from hangar support
        (-195.25 + (DIMENSIONS::CHASSIS_LENGTH.value() * 2)) / 12.0,
        46 / 12.0,
        0,
        "WPT_Shoot_P_1"
    }; // use central_waypoint as well?
    // const field_pose LAUNCH_PAD_WAYPOINT2{(-195.25 / 12.0) + (DIMENSIONS::CHASSIS_LENGTH * 2), 46 / 12.0, 0, "WPT_Shoot_P_2"};


    const field_pose IMPORTANT_WAYPOINT_EXAMPLE{0, 0, 0, "Waypoint Example"}; // could be a start or end position, but also a useful waypoint for a trajectory where toTranslation is used
    const field_pose CENTRAL_WAYPOINT{ // Good place to put as a waypoint to not hit the hangar and for good lines to other places
        -200.0 / 12.0,
        -65.0 / 12.0,
        0,
        "WPT_Central"
    };
}