// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The RobotMap class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {
    // Controller
    public static final int DRIVE_CONTROLLER_CHANNEL = 0;

    // Motor IDs
    public static final int[] LEFT_FRONT_IDS = {1, 2};
    public static final int[] LEFT_REAR_IDS = {3, 4};
    public static final int[] RIGHT_FRONT_IDS = {5, 6};
    public static final int[] RIGHT_REAR_IDS = {7, 8};

    // Encoder IDs
    public static final int[] LEFT_FRONT_ENCODER_IDS = {0, 1};
    public static final int[] LEFT_REAR_ENCODER_IDS = {2, 3};
    public static final int[] RIGHT_FRONT_ENCODER_IDS = {4, 5};
    public static final int[] RIGHT_REAR_ENCODER_IDS = {6, 7};

    // Encoder Conversion
    public static final double DRIVE_ENCODER_FEET_PER_TICK = 1; // 1 / (gearRatio * wheelCircumference * 12)
    public static final double ANGLE_ENCODER_RADIAN_PER_PULSE = 1; // 1 / (pulse_per_motor_rot * gearRatio * 2 * PI)
}
