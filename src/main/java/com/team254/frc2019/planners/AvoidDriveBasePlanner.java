package com.team254.frc2019.planners;

import com.team254.frc2019.states.SuperstructureGoal;

/**
 * Superstructure motion planner meant to avoid colliding with the drivebase
 */
public class AvoidDriveBasePlanner implements ISuperstructureMotionPlanner {
    public boolean isValidGoal(SuperstructureGoal goal) {
        return true;
    }

    @Override
    public SuperstructureGoal plan(SuperstructureGoal prevSetpoint, SuperstructureGoal goal) {
        // No restrictions needed.
        return new SuperstructureGoal(goal.state);
    }
}
