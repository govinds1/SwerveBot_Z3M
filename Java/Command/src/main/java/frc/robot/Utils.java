package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// contains helpful utility functions
public class Utils {
    public static Translation2d TranslationMetersToFeet(Translation2d translationMeters) {
        return new Translation2d(Units.metersToFeet(translationMeters.getX()), Units.metersToFeet(translationMeters.getY()));
    }

    public static Pose2d PoseMetersToFeet(Pose2d poseMeters) {
        return new Pose2d(TranslationMetersToFeet(poseMeters.getTranslation()), poseMeters.getRotation());
    }

    public static double Deadband(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        int sign = (value < 0) ? -1 : 1;
        return sign * (Math.abs(value) - deadzone);
    }

    public static double PowerKeepSign(double value, double power) {
        return value * Math.abs(Math.pow(value, power - 1));
    }
}
