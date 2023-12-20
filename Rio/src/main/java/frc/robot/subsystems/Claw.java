package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.supers.ClawSuper;

public class Claw extends ClawSuper {

    //Spark Maxes
    public static CANSparkMax rightRollers = new CANSparkMax(DriveConstants.kRightRollersPort, MotorType.kBrushless);
    public static CANSparkMax leftRollers = new CANSparkMax(DriveConstants.kLeftRollersPort, MotorType.kBrushless);
    //Solenoids
    DoubleSolenoid m_doubleSolenoid = RobotContainer.m_pH.makeDoubleSolenoid(DriveConstants.kForwardClawPort, DriveConstants.kReverseClawPort);

    //Open status boolean for claw
    public static boolean isOpen = true;

    public Claw() {
        //Settings for sparks
        rightRollers.restoreFactoryDefaults();
        leftRollers.restoreFactoryDefaults();

        leftRollers.follow(rightRollers, true);

        //Settings for solenoid
        toggleClaw(false);
    }

    /**
     * Rollers speed - Rotations/Minute
     */
    public void setRollers(double speed) {
        rightRollers.set(speed);
        SmartDashboard.putNumber("Claw/Roller P. Out", speed);
    }

    /**
     * Open and closing the claw
     */
    public void toggleClaw(boolean open) {
        if (open) {
            m_doubleSolenoid.set(Value.kForward);
        } else {
            m_doubleSolenoid.set(Value.kReverse);
        }
    }

    /**
     * Changes if the claw being open is false or true
     */
    public void toggleClaw() {
        isOpen = !isOpen;
        toggleClaw(isOpen);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
