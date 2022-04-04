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
    const int SHOOTER = 9;
    const int TURRET = 10;
}

namespace DIMENSIONS {
    const units::length::inch_t WHEELBASE_LENGTH = 1.0_in; // From center of back wheel to center of front wheel, in inches
    const units::length::inch_t WHEELBASE_WIDTH = 1.0_in; // From center of left wheel to center of right wheel, in inches
    const units::length::inch_t CHASSIS_LENGTH = 1.0_in; // Entire chassis length, including bumpers, in inches
    const units::length::inch_t CHASSIS_WIDTH = 1.0_in; // Entire chassis width, including bumpers, in inches
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
        double F;
        pid_config(double p, double i, double d, double f) {
            P = p;
            I = i;
            D = d;
            F = f;
        }
    };
    // Wheels
    const pid_config DRIVE{0, 0, 0, 0};
    const pid_config ANGLE{0, 0, 0, 0};

    // Chassis (for path following)
    const pid_config CHASSIS_X{0, 0, 0, 0};
    const pid_config CHASSIS_Y{0, 0, 0, 0};
    const pid_config CHASSIS_ROT{0, 0, 0, 0};

    const pid_config SHOOTER{0.5, 0, 0, 0.4};
    const pid_config TURRET{0, 0, 0, 0};
}

namespace SPEEDS {
    const units::velocity::feet_per_second_t MAX_FORWARD_SPEED = 1.0_fps;
    const units::velocity::feet_per_second_t MAX_STRAFE_SPEED = 1.0_fps;
    const units::radians_per_second_t MAX_TURN_SPEED = units::radians_per_second_t(1.0);

    const units::velocity::feet_per_second_t MAX_CHASSIS_SPEED = MAX_FORWARD_SPEED + MAX_STRAFE_SPEED;
    const units::acceleration::feet_per_second_squared_t MAX_CHASSIS_ACCEL = units::acceleration::feet_per_second_squared_t(1);
    const units::angular_velocity::radians_per_second_t MAX_CHASSIS_TURN_SPEED = 6.28_rad_per_s;
    const auto MAX_CHASSIS_TURN_ACCEL= 3.14_rad_per_s / 1_s;
}

// Information needed for TrajectoryGenerator
namespace TRAJECTORIES {
    const frc::Pose2d trajectoryTolerance{units::foot_t(0.05), units::foot_t(0.05), frc::Rotation2d(units::radian_t(0.05))};
    // All trajectories are set in PathManager constructor
}

namespace AUTON {
    // Naming convention (no spaces): X-Y
    // X = starting position -> {Left, Middle, Right}
    // Y = # of balls -> {1, 2, 3, 4, 5, ?}
    const std::vector<std::string> AUTO_LIST = {"Left-1", "Left-2", "Left-3", "Left-4", "Left-5"};
    const units::time::second_t SHOOT_TIME = 1.5_s;
    const units::time::second_t INTAKE_TIME = 1.0_s;
}

namespace TALONFX {
    // talon sensor velocity is measured as ticks per 100ms
    const double TICKS_PER_ROTATION = 2048;
    const double _100MS_PER_MIN = 10 * 60;
    const double TALON_RPM_CONVERSION = (1 / TICKS_PER_ROTATION) * _100MS_PER_MIN; // without gear ratio -> sensor velocity to motor rpm
}

namespace SHOOTER {
    const double SHOOTING_RPM = 4000;
    const double SHOOTER_GEAR_RATIO = 10.71;
    const double TURRET_GEAR_RATIO = 10.71;
    const double SHOOTER_RPM_CONVERSION = TALONFX::TALON_RPM_CONVERSION / SHOOTER_GEAR_RATIO;
    const double TURRET_RPM_CONVERSION = TALONFX::TALON_RPM_CONVERSION / TURRET_GEAR_RATIO;

    // PID constants in PID_VALUES
}