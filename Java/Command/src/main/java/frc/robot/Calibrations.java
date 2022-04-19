package frc.robot;


public final class Calibrations {
    // PID values
    public static final double[] DRIVE_PID_VALUES = {0.0, 0.0, 0.0};
    public static final double[] ANGLE_PID_VALUES = {0.0, 0.0, 0.0};

    // Wheel angle zeros
    public static int LEFT_FRONT_ZERO = 0;
    public static int LEFT_REAR_ZERO = 0;
    public static int RIGHT_FRONT_ZERO = 0;
    public static int RIGHT_REAR_ZERO = 0;

    // Encoder Conversion
    public static final double DRIVE_ENCODER_FEET_PER_TICK = 1; // 1 / (gearRatio * wheelCircumference * 12)
    public static final double ANGLE_ENCODER_RADIAN_PER_PULSE = 1; // 1 / (pulse_per_motor_rot * gearRatio * 2 * PI)

    // Dimensions
    public static final double WHEELBASE_LENGTH = 1.0; // From center of back wheel to center of front wheel, in inches
    public static final double WHEELBASE_WIDTH = 1.0; // From center of left wheel to center of right wheel, in inches
    public static final double CHASSIS_LENGTH = 1.0; // Entire chassis length, including bumpers, in inches
    public static final double CHASSIS_WIDTH = 1.0; // Entire chassis width, including bumpers, in inches

    // Speeds
    // all drive speeds are in feet per second
    // all angular speeds are in radians per second
    public static final double MAX_FORWARD_SPEED = 1.0; // feet per sec
    public static final double MAX_STRAFE_SPEED = 1.0; // feet per sec
    public static final double MAX_TURN_SPEED = 1.0; // rads per sec

    public static final double MAX_CHASSIS_SPEED = MAX_FORWARD_SPEED + MAX_STRAFE_SPEED; // feet per sec
    public static final double MAX_CHASSIS_ACCEL = 1.0; // feet per sec^2
    public static final double MAX_CHASSIS_TURN_SPEED = 6.28; // rads per sec
    public static final double MAX_CHASSIS_TURN_ACCEL= 3.14; // rads per sec^2
}
