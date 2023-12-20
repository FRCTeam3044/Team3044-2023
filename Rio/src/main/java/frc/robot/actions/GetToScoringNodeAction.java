package frc.robot.actions;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.RobotContainer;
import frc.robot.commands.PickupCubeCommand;

public class GetToScoringNodeAction implements Action {
    private final PickupCubeCommand m_pickupCubeCommand;
    private final Pose3d targetNode;
    private boolean started = false;
    private boolean done = false;

    public GetToScoringNodeAction(PickupCubeCommand pickupCubeCommand, Pose3d targetNode) {
        m_pickupCubeCommand = pickupCubeCommand;
        this.targetNode = targetNode;
    }

    @Override
    public void start() {
        started = true;
        m_pickupCubeCommand.setCubeCenter(targetNode);
        m_pickupCubeCommand.setScoring(true);
        m_pickupCubeCommand.schedule();
    }

    @Override
    public void end() {
        m_pickupCubeCommand.cancel();
        RobotContainer.singleton.m_followTrajectory.cancel();
        done = true;
    }

    @Override
    public boolean isDone() {
        return done || (started && m_pickupCubeCommand.isFinished());
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    public CANCEL_CONDITION getCancelCondition() {
        return CANCEL_CONDITION.DRIVE;
    }
}