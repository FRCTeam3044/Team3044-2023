package frc.robot.supers;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.packages.InverseKinematicsRER.ArmPosition;
import frc.robot.Constants.ArmConstants;


public abstract class ArmSuper extends SubsystemBase {
    public abstract void resetTranslation();

    public abstract void setArmMechanism(ArmPosition armMechanism);

    public abstract void execute();

    public double SHOULDER_IN_BOT_RANGE = 30;
    // Telescope constants
    public double MAX_TELE_ERROR = 0.01;
    // Old speed: 0.5
    public double TELE_MOVE_SPEED = 0.4;

    // Wrist constants (in degrees)
    public double WRIST_SLOWDOWN_THRESHOLD = 15;
    public double WRIST_STOP_THRESHOLD = 3;
    // In motor percent output
    // Old value: 0.5
    public double WRIST_MOVEMENT_SPEED = 0.5;

    public Mechanism2d armMechanism = new Mechanism2d(3, 1.5);
    public MechanismLigament2d shoulderMechanism;
    public MechanismLigament2d telescopeMechanism;
    public MechanismLigament2d wristMechanism;
    public Mechanism2d targetArmMechanism = new Mechanism2d(3, 1.5);
    public MechanismLigament2d targetShoulderMechanism;
    public MechanismLigament2d targetTelescopeMechanism;
    public MechanismLigament2d targetWristMechanism;

    public ArmPosition targetPosition = new ArmPosition(-90, ArmConstants.minTranslation, 90);

    public abstract boolean armInPlace();

    public abstract void enableTranslationWait();
}
