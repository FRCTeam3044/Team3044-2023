package frc.packages.InverseKinematicsRER;

public class ArmPosition {
    public double shoulder;
    public double telescope;
    public double wrist;

    public ArmPosition(double shoulder, double telescope, double wrist){
        this.wrist = wrist;
        this.shoulder = shoulder;
        this.telescope = telescope;
    }
}
