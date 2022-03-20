#pragma once


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

namespace PID_VALUES { // Might have to make separate values for each wheel?
    const double DRIVE_P = 0.0;
    const double DRIVE_I = 0.0;
    const double DRIVE_D = 0.0;

    const double ANGLE_P = 0.0;
    const double ANGLE_I = 0.0;
    const double ANGLE_D = 0.0;
}

namespace SPEEDS {
    const units::velocity::feet_per_second_t MAX_FORWARD_SPEED = 1.0_fps;
    const units::velocity::feet_per_second_t MAX_STRAFE_SPEED = 1.0_fps;
    const double MAX_TURN_SPEED = 1.0; // Doesn't work as units::radians_per_second_t for some reason
}


// For correcting/setting odometry to certain known locations when possible
// Examples: Auton starts, scoring locations, corners of the field, other landmarks, etc
// as a side note, it is possible to do a more dynamic calibration with vision, so adding in vision target locations is helpful as well
namespace POSES {
    struct field_pose {
        double x; // across length of field, postive towards forward (to opponent's alliance station)
        double y; // across width of field, positive towards left
        double rotation; // where 0 is facing the opponent's alliance station, positive towards turning left (CCW)
    };
    // (0, 0, 0) means robot is in the center of the field, facing the opponent's alliance station
    // ^^^^^^^^^ or we change the world coordinates to whatever we want ^^^^^^^^^

    const field_pose AUTON_LEFT_START {3.0, 10.0, -30};
}