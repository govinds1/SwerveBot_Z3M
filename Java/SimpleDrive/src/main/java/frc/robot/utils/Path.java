package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class Path {
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
    private State start;
    private State goal;
    private State[] waypoints;

    // Initializes a Path with start and goal states and any number of intermediate waypoints
    public Path(State start, State goal, State... waypoints) {
        this.start = start;
        this.goal = goal;
        this.waypoints = waypoints;
    }

    public List<State> GetStates() {
        List<State> states = new ArrayList<State>();
        states.add(start);
        for (State s : waypoints) {
            states.add(s);
        }
        states.add(goal);
        return states;
    }
}
