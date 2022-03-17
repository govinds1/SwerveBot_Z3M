#pragma once

namespace MOTOR_CAN_ID {
    public struct wheel_module {
        int DRIVE_ID;
        int ANGLE_ID;
        wheel_module(int drive_id, int angle_id) {
            DRIVE_ID = drive_id;
            ANGLE_ID = angle_id;
        }
    }

    public static final wheel_module LEFT_FRONT{1, 2};
    public static final wheel_module LEFT_REAR{3, 4};
    public static final wheel_module RIGHT_FRONT{5, 6};
    public static final wheel_module RIGHT_REAR{7, 8};
}