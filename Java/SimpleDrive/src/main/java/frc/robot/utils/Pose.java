package frc.robot.utils;

public class Pose {
    // Warning: This coordinate system is different than the robot control coordinate system in the swerve drive math
    // x is +forward (to opponent's alliance station)
    // y is +left
    // theta is +CCW (turning left)
    public double x;
    public double y;
    public double theta;

    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double DistanceTo(Pose other, boolean checkRotation) {
        if (checkRotation) {
            return Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2) + Math.pow(other.theta - this.theta, 2));
        }
        return Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
    }

    public boolean EqualTo(Pose other, double threshold, boolean checkRotation) {
        return DistanceTo(other, checkRotation) < threshold;
    }

    public ChassisSpeeds SpeedsTo(Pose other, double dt) {
        double vx = (other.x - this.x) / dt;
        double vy = (other.y - this.y) / dt;
        double omega = (other.theta - this.theta) / dt;
        return new ChassisSpeeds(vx, vy, omega);
    }
}
