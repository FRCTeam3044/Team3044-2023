package frc.robot.subsystems_sim;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.packages.InverseKinematicsRER.ArmPosition;
import frc.robot.Constants;
import frc.robot.supers.ArmSuper;

public class ArmSim extends ArmSuper {
    public ArmSim() {
        MechanismRoot2d armRoot = targetArmMechanism.getRoot("arm", 1.525, Constants.ArmConstants.shoulderHeight);
        targetShoulderMechanism = armRoot.append(
            new MechanismLigament2d("shoulder", 0, 90));
        targetTelescopeMechanism = targetShoulderMechanism.append(
            new MechanismLigament2d("telescope", Constants.ArmConstants.minTranslation, 0, 5, new Color8Bit(255, 0, 0)));
        targetWristMechanism = targetTelescopeMechanism.append(
            new MechanismLigament2d("wrist", 0.386, 0, 5, new Color8Bit(0, 255, 0)));
    }

    public void setArmMechanism(ArmPosition armMechanism) {
        targetShoulderMechanism.setAngle(armMechanism.shoulder);
        targetTelescopeMechanism.setLength(armMechanism.telescope);
        targetWristMechanism.setAngle(armMechanism.wrist);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("TargetArm", targetArmMechanism);
    }

    @Override
    public void resetTranslation() {
        setArmMechanism(new ArmPosition(shoulderMechanism.getAngle(), Constants.ArmConstants.minTranslation, wristMechanism.getAngle()));
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean armInPlace() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void enableTranslationWait() {
        // TODO Auto-generated method stub
        
    }
}
