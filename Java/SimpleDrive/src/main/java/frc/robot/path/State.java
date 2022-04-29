package frc.robot;

public class State {
    // Warning: This coordinate system is different than the robot control coordinate system in the swerve drive math
    // x is +forward (to opponent's alliance station)
    // y is +left
    // theta is +CCW (turning left)
    private double x;
    private double y;
    private double theta;

    public State(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double DistanceTo(State other, boolean checkRotation) {
        if (checkRotation) {
            return Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2) + Math.pow(other.theta - this.theta, 2));
        }
        return Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
    }

    public boolean EqualTo(State other, double threshold, boolean checkRotation) {
        return DistanceTo(other, checkRotation) < threshold;
    }
}