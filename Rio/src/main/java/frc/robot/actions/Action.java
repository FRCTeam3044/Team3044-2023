package frc.robot.actions;

// A simple interface to allow for more complex, high level actions
// to be created and scheduled.
public interface Action {
    void start();

    void end();

    boolean isDone();

    boolean isStarted();

    CANCEL_CONDITION getCancelCondition();

    default void execute() {
    }

    default void alertManual() {
    }

    // Important to note that drivers can still manually cancel actions regardless of this
    enum CANCEL_CONDITION {
        // The action won't be automatically cancelled
        NONE,
        // The action will be cancelled if field relative drive is enabled
        DRIVE,
        // The action will be cancelled if the bot is rotated during field relative drive
        ROTATE,
    }

    interface ActionGroup extends Action {
        Action[] getActions();

        default void cancelOnCondition(CANCEL_CONDITION condition) {
            for (Action action : getActions()) {
                if (action.getCancelCondition() == condition) action.end();
            }
        }

        default void alertManual() {
            for (Action action : getActions()) {
                action.alertManual();
            }
        }

        default void end() {
            for (Action action : getActions()) {
                action.end();
            }
        }
    }
}