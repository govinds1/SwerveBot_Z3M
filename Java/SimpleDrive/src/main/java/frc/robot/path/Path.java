package frc.robot.path;

public class Path {
    private State start;
    private State goal;
    private State[] waypoints;

    // Initializes a Path with start and goal states and any number of intermediate waypoints
    public Path(State start, State goal, State... waypoints) {
        this.start = start;
        this.goal = goal;
        this.waypoints = waypoints;
    }
}
