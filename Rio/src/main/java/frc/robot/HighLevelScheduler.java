package frc.robot;

import java.util.ArrayList;

import frc.robot.actions.Action;
import frc.robot.actions.Action.CANCEL_CONDITION;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.FieldRelative;

public class HighLevelScheduler {
    public ArrayList<Action> actions = new ArrayList<>();

    private final FieldRelative m_fieldRelative;
    private final ArcadeDrive m_arcadeDrive;

    public HighLevelScheduler(FieldRelative fieldRelative, ArcadeDrive arcadeDrive) {
        m_fieldRelative = fieldRelative;
        m_arcadeDrive = arcadeDrive;
    }

    public void add(Action action) {
        if (actions.size() < 3) actions.add(action);
    }

    public void removeLast() {
        if (actions.size() > 1) actions.get(0).end();
        if (!actions.isEmpty()) actions.remove(actions.size() - 1);
    }

    public void skip() {
        if (!actions.isEmpty()) {
            actions.get(0).end();
            actions.remove(0);
        }
    }

    public void cancelAll() {
        if (!actions.isEmpty()) {
            actions.get(0).end();
        }
        actions.clear();
    }

    public void execute() {
        // for(Action a : actions){
        //     System.out.println(a.getClass().getSimpleName());
        // }
        if (!actions.isEmpty()) {
            if (actions.get(0).isDone()) {
                actions.get(0).end();
                actions.remove(0);
                if (!actions.isEmpty()) actions.get(0).start();
            } else if (!actions.get(0).isStarted()) {
                actions.get(0).start();
            }
        }
        handleAutoCanceling();
        if (!actions.isEmpty()) actions.get(0).execute();
    }

    private void handleAutoCanceling() {
        if (m_arcadeDrive.isScheduled()) {
            cancelAll();
        } else if (m_fieldRelative.isScheduled()) {
            cancelOnCondition(CANCEL_CONDITION.DRIVE);
            for (Action a : actions) {
                a.alertManual();
            }
            if (m_fieldRelative.manualRot) {
                cancelOnCondition(CANCEL_CONDITION.ROTATE);
            }
        }
    }

    private void cancelOnCondition(CANCEL_CONDITION condition) {
        if (actions.size() < 1) return;
        if (actions.get(0).getCancelCondition() == condition) actions.get(0).end();
        for (Action a : actions) {
            if (a instanceof Action.ActionGroup) {
                ((Action.ActionGroup) a).cancelOnCondition(condition);
            }
        }
        actions.removeIf(a -> a.getCancelCondition() == condition);
    }
}
