#pragma once

#include <vector>
#include <string>
#include <utility>


#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace MOTOR_CAN_ID {
    struct wheel_module {
        int DRIVE_ID;
        int ANGLE_ID;
        wheel_module(int drive_id, int angle_id) {
            DRIVE_ID = drive_id;
            ANGLE_ID = angle_id;
        }
    };

    const wheel_module LEFT_FRONT{1, 2};
    const wheel_module LEFT_REAR{3, 4};
    const wheel_module RIGHT_FRONT{5, 6};
    const wheel_module RIGHT_REAR{7, 8};
}

namespace DIMENSIONS {
    const units::length::foot_t WHEELBASE_LENGTH = 1.0_ft; // From center of back wheel to center of front wheel, in inches
    const units::length::foot_t WHEELBASE_WIDTH = 1.0_ft; // From center of left wheel to center of right wheel, in inches
}

namespace ENCODER_CONVERSIONS {
    const double DRIVE_FEET_PER_TICK = 1.0; // 1 / (gearRatio * wheelCircumference * 12)
    const double DRIVE_FEET_PER_SECOND_PER_TICK = DRIVE_FEET_PER_TICK / 60;// 1 / (gearRatio * wheelCircumference * 12 * 60)
    const double ANGLE_RAD_PER_TICK = 1.0; // 1 / (gearRatio * 2 * PI)
}

namespace ENCODER_ZEROS {
    // const int LEFT_FRONT = frc::SmartDashboard::GetNumber("Wheel Module/Left Front/Angle/Encoder Zero", 0);
    // const int LEFT_REAR = frc::SmartDashboard::GetNumber("Wheel Module/Left Rear/Angle/Encoder Zero", 0);
    // const int RIGHT_FRONT = frc::SmartDashboard::GetNumber("Wheel Module/Right Front/Angle/Encoder Zero", 0);
    // const int RIGHT_REAR = frc::SmartDashboard::GetNumber("Wheel Module/Right Rear/Angle/Encoder Zero", 0);

    // defined in Constants.cpp
    extern int LEFT_FRONT;
    extern int LEFT_REAR;
    extern int RIGHT_FRONT;
    extern int RIGHT_REAR;
    // int LEFT_FRONT = 0;
    // int LEFT_REAR = 0;
    // int RIGHT_FRONT = 0;
    // int RIGHT_REAR = 0;
}

namespace PID_VALUES { // Might have to make separate values for each wheel?
    struct pid_config {
        double P;
        double I;
        double D;
        pid_config(int p, int i, int d) {
            P = p;
            I = i;
            D = d;
        }
    };
    // Wheels
    const pid_config DRIVE{0, 0, 0};
    const pid_config ANGLE{0, 0, 0};

    // Chassis (for path following)
    const pid_config CHASSIS_X{0, 0, 0};
    const pid_config CHASSIS_Y{0, 0, 0};
    const pid_config CHASSIS_ROT{0, 0, 0};
}

namespace SPEEDS {
    const units::velocity::feet_per_second_t MAX_FORWARD_SPEED = 1.0_fps;
    const units::velocity::feet_per_second_t MAX_STRAFE_SPEED = 1.0_fps;
    const double MAX_TURN_SPEED = 1.0; // Doesn't work as units::radians_per_second_t for some reason

    const units::velocity::feet_per_second_t MAX_CHASSIS_SPEED = MAX_FORWARD_SPEED + MAX_STRAFE_SPEED;
    const units::acceleration::feet_per_second_squared_t MAX_CHASSIS_ACCEL = units::acceleration::feet_per_second_squared_t(1);
    const units::angular_velocity::radians_per_second_t MAX_CHASSIS_TURN_SPEED = 6.28_rad_per_s;
    const auto MAX_CHASSIS_TURN_ACCEL= 3.14_rad_per_s / 1_s;
}


// For correcting/setting odometry to certain known locations when possible
// Examples: Auton starts, scoring locations, corners of the field other landmarks, etc
// as a side note, it is possible to do a more dynamic calibration with vision, so adding in vision target locations is helpful as well
// Also used for spline trajectories, as start/end locations or waypoints
// https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022LayoutMarkingDiagram.pdf
namespace POSES {
    struct field_pose {
        double X; // across length of field, postive towards forward (to opponent's alliance station)
        double Y; // across width of field, positive towards left
        double ROT; // where 0 is facing the opponent's alliance station, positive towards turning left (CCW)
        std::string NAME;

        frc::Translation2d toTranslation() const {
            return frc::Translation2d(units::foot_t(X), units::foot_t(Y));
        }
        frc::Pose2d toPose() const {
            return frc::Pose2d(this->toTranslation(), frc::Rotation2d(units::angle::radian_t(ROT)));
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

    const field_pose AUTON_LEFT_START {-3.0, 3.0, -1.57, "Left Start"};
    const field_pose AUTON_MIDDLE_START {-4.0, 0, 0, "Middle Start"};
    const field_pose AUTON_RIGHT_START {-3.0, -3.0, 1.57, "Right Start"};

    // For try hard auton where you just "recursively" go through balls
    // const field_pose BALL_1{0, 0, 0, "Ball 1"}; // Far right ball
    // const field_pose BALL_2{0, 0, 0, "Ball 2"}; // Right side ball closer to center
    // const field_pose BALL_3{0, 0, 0, "Ball 3"}; // Human player station
    // const field_pose BALL_4{0, 0, 0, "Ball 4"}; // Far left ball

    // Set these Poses to be where you want the ROBOT to be, not exactly on the ball! So that intake can face it and drive forward a little to pick it up
    // ex. Right Ball should be set so the robot intake faces the right wall and is a little to the left of where the ball actually sits
    const field_pose BALL_RIGHT{-30 / 12.0, -145 / 12.0, -90 * M_PI / 180.0, "Right Ball"}; // Ball on far right
    const field_pose BALL_MIDDLE{-112 / 12.0, -82 / 12.0, 180.0, "Middle Ball"}; // Ball in center-ish
    const field_pose BALL_LEFT{-112 / 12.0, 82 / 12.0, 180.0, "Left Ball"}; // Ball on left
    const field_pose BALL_HUMAN_PLAYER{-256 / 12.0, -112 / 12.0, -133.75 * M_PI / 180.0, "Human Player Ball"}; // Ball by human player station

    // these might be the same as a ball pose
    // these should be exact where you want, face the hub but no need to come at it from a certain side
    const field_pose SHOOTING_SPOT_LEFT{0, 0, 0, "Shoot Left"};  
    const field_pose SHOOTING_SPOT_MIDDLE{0, 0, 0, "Shoot Middle"};
    const field_pose SHOOTING_SPOT_RIGHT{0, 0, 0, "Shoot Right"};


    const field_pose IMPORTANT_WAYPOINT_EXAMPLE{0, 0, 0, "Waypoint Example"}; // could be a start or end position, but also a useful waypoint for a trajectory where toTranslation is used
    const field_pose CENTRAL_WAYPOINT{-200.0 / 12.0, -65.0 / 12.0, 0, "Central Waypoint"}; // Good place to put as a waypoint to not hit the hangar and for good lines to other places
}

// Information needed for TrajectoryGenerator
namespace TRAJECTORIES {
    const frc::Pose2d trajectoryTolerance{units::foot_t(0.05), units::foot_t(0.05), frc::Rotation2d(units::radian_t(0.05))};

    // Full names not necessary, they are created from field_pose names
    // const std::string LEFT_START_TO_BALL_1 = "Left Start to Ball 1";
    // const std::string BALL_1_TO_BALL_2 = "Ball 1 to Ball 2";
    // const std::string LEFT_START_TO_BALL_2 = "Left Start to Ball 2";
    // const std::string BALL_2_TO_BALL_3 = "Ball 2 to Ball 3";
    // const std::string BALL_3_TO_BALL_4 = "Ball 3 to Ball 4";
    // const std::string BALL_4_TO_BALL_5 = "Ball 4 to Ball 5";
    // const std::string BALL_5_TO_SHOOTING_SPOT_LEFT = "Ball 5 to Shooting Spot Left";
    // const std::string BALL_5_TO_SHOOTING_SPOT_MIDDLE = "Ball 5 to Shooting Spot Middle";
    // const std::string BALL_5_TO_SHOOTING_SPOT_RIGHT = "Ball 5 to Shooting Spot Right";

}

namespace AUTON {
    // Naming convention (no spaces): X-Y
    // X = starting position -> {Left, Middle, Right}
    // Y = # of balls -> {1, 2, 3, 4, 5, ?}
    const std::vector<std::string> AUTO_LIST = {"Left-1", "Left-2", "Left-3", "Left-4", "Left-5"};
    const units::time::second_t SHOOT_TIME = 1.5_s;
    const units::time::second_t INTAKE_TIME = 1.0_s;
}