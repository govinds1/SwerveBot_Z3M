package frc.robot.utils;

import java.util.List;

import frc.robot.utils.Path.State;

public class HolonomicPathFollower {
    private Path currentPath;
    private List<State> nextStates;
    private State lastState;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController thetaPID;
    private double dt;
    
    // Initializes HolonomicPathFollower with a path to follow
    public HolonomicPathFollower(Path pathToFollow, PIDController xPID, PIDController yPID, PIDController thetaPID) {
        currentPath = pathToFollow;
        nextStates = pathToFollow.GetStates();
        this.xPID = xPID;
        this.yPID = yPID;
        this.thetaPID = thetaPID;
        dt = 0.02;
    }

    public void SetPeriod(double delta_t) {
        dt = delta_t;
    }

    public void StartPath(Path newPath) {
        currentPath = newPath;
        nextStates = newPath.GetStates();
    }

    // Calculate Speeds to the next state in the current path given a current drive State
    // Must be called every cycle while following the path
    public ChassisSpeeds CalculateSpeeds(State currentState) {
        while (!AtNextPathState(currentState)) {
            nextStates.remove(0);
        }
        State targetState = nextStates.get(0);
        if (lastState == null) {
            lastState = currentState;
        }
        ChassisSpeeds currentSpeeds = lastState.SpeedsTo(currentState, dt);
        double vx = xPID.Calculate(currentState.x, targetState.x, currentSpeeds.vx);
        double vy = yPID.Calculate(currentState.y, targetState.y, currentSpeeds.omega);
        double omega = thetaPID.Calculate(currentState.theta, targetState.theta, currentSpeeds.omega);
        lastState = currentState;

        return new ChassisSpeeds(vx, vy, omega);
    }

    public boolean AtNextPathState(State currentState) {
        return currentState.EqualTo(nextStates.get(0), 0.01, true);
    }
}
