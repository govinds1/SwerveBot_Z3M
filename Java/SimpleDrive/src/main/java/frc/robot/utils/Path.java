package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class Path {
    private Pose start;
    private Pose goal;
    private Pose[] waypoints;
    private String name;

    // Initializes a Path with start and goal states and any number of intermediate waypoints
    public Path(String name, Pose start, Pose goal, Pose... waypoints) {
        this.name = name;
        this.start = start;
        this.goal = goal;
        this.waypoints = waypoints;
    }

    public List<Pose> GetStates() {
        List<Pose> states = new ArrayList<Pose>();
        states.add(start);
        for (Pose s : waypoints) {
            states.add(s);
        }
        states.add(goal);
        return states;
    }

    public String GetName() {
        return name;
    }
}
