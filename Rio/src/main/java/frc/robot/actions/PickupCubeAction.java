package frc.robot.actions;

import frc.robot.commands.PickupCubeCommand;

public class PickupCubeAction implements Action {
    private PickupCubeCommand m_pickupCubeCommand;

    private boolean started = false;

    public PickupCubeAction(PickupCubeCommand pickupCubeCommand) {
        m_pickupCubeCommand = pickupCubeCommand;
    }

    @Override
    public void start() {
        started = true;
        m_pickupCubeCommand.schedule();
    }

    @Override
    public boolean isDone() {
        return started && !m_pickupCubeCommand.isScheduled();
    }

    public void end() {
        m_pickupCubeCommand.cancel();
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    @Override
    public CANCEL_CONDITION getCancelCondition() {
        return CANCEL_CONDITION.DRIVE;
    }
}
