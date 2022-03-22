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
    int LEFT_FRONT = 0;
    int LEFT_REAR = 0;
    int RIGHT_FRONT = 0;
    int RIGHT_REAR = 0;
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
namespace POSES {
    struct field_pose {
        double X; // across length of field, postive towards forward (to opponent's alliance station)
        double Y; // across width of field, positive towards left
        double ROT; // where 0 is facing the opponent's alliance station, positive towards turning left (CCW)

        frc::Translation2d toTranslation() const {
            return frc::Translation2d(units::foot_t(X), units::foot_t(Y));
        }
        frc::Pose2d toPose() const {
            return frc::Pose2d(this->toTranslation(), frc::Rotation2d(units::angle::radian_t(ROT)));
        }
        field_pose(double x, double y, double rot) {
            X = x;
            Y = y;
            ROT = rot;
        }
    };
    // (0, 0, 0) means robot is in the center of the field, facing the opponent's alliance station
    // ^^^^^^^^^ or we change the world coordinates to whatever we want ^^^^^^^^^

    const field_pose AUTON_LEFT_START {3.0, 10.0, -30.0*M_PI/180.0};
    const field_pose BALL_1{0, 0, 0};
    const field_pose BALL_2{0, 0, 0};
}

// Information needed for TrajectoryGenerator
namespace TRAJECTORIES {
    const frc::Pose2d trajectoryTolerance{units::foot_t(0.05), units::foot_t(0.05), frc::Rotation2d(units::radian_t(0.05))};

    // Add names here?
}