package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.packages.InverseKinematicsRER.ArmPosition;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.supers.ArmSuper;
import org.littletonrobotics.junction.Logger;

public class Arm extends ArmSuper {
    public CANSparkMax armJoint1 = new CANSparkMax(DriveConstants.kShoulderPort1, MotorType.kBrushless);
    public CANSparkMax armJoint2 = new CANSparkMax(DriveConstants.kShoulderPort2, MotorType.kBrushless);
    public TalonSRX armExtend = new TalonSRX(DriveConstants.kTelescopePort);
    public TalonSRX wrist = new TalonSRX(DriveConstants.kWristPort);

    // Positive offset
    public static AnalogPotentiometer shoulderPot = new AnalogPotentiometer(0, -3600, 1607.7);
    private LinearFilter shoulderPotFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // Negative offset
    public static AnalogPotentiometer wristPot = new AnalogPotentiometer(1, 3600, -1587.5);
    private LinearFilter wristPotFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    //public static AHRS wristNavX = new AHRS(I2C.Port.kMXP);
    // TODO: we want to flip this
    public DigitalInput translationLimit = new DigitalInput(1);

    // Shoulder constants 
    // Old value: 200
    public final double SHOULDER_MAX_ACCEL = 200;
    // Old value: 400
    public final double SHOULDER_MAX_VEL = 400;
    public final double SHOULDER_MIN_VEL = 0;
    public final double SHOULDER_MAX_ERROR = 0;
    public final double SHOULDER_FF = 0.00156;
    // Defines, in degrees, the range around 90 degrees where the shoulder is considered to be in the robot
    // TODO: Seperate this for forward and back

    // The wrist will not go out of this range (90 degrees + or - this number) while the shoulder is inside the robot
    public final double WRIST_SAFE_RANGE = 15;

    public boolean translationZeroed = false;
    private double extEncoderOffset = 0;
    private boolean translationLimitSwitch = false;
    private boolean translationOffsetUpdated = false;

    private double shoulderAnglePot = 0;

    private ArmPosition currentPosition = new ArmPosition(90, ArmConstants.minTranslation, 90);

    private boolean translationWait = false;
    private ArmPosition translationWaitTarget = new ArmPosition(-1,-1,-1);
    

    public Arm() {

        armJoint1.restoreFactoryDefaults();
        armJoint2.restoreFactoryDefaults();
        armExtend.configFactoryDefault();
        wrist.configFactoryDefault();

        armExtend.setNeutralMode(NeutralMode.Brake);
        armJoint1.setIdleMode(IdleMode.kBrake);
        armJoint2.setIdleMode(IdleMode.kBrake);
        wrist.setNeutralMode(NeutralMode.Brake);

        armJoint2.follow(armJoint1, true);

        armJoint1.getEncoder().setPosition(0);

        SparkMaxPIDController armPID = armJoint1.getPIDController();
        armPID.setSmartMotionMaxVelocity(SHOULDER_MAX_VEL, 0);
        armPID.setSmartMotionMaxAccel(SHOULDER_MAX_ACCEL, 0);
        armPID.setSmartMotionMinOutputVelocity(SHOULDER_MIN_VEL, 0);
        armPID.setSmartMotionAllowedClosedLoopError(SHOULDER_MAX_ERROR, 0);
        armPID.setFF(SHOULDER_FF);

        // 3d renderer support
        MechanismRoot2d armRoot = armMechanism.getRoot("arm", 1.5, Constants.ArmConstants.shoulderHeight);
        shoulderMechanism = armRoot.append(
            new MechanismLigament2d("shoulder", 0, -90));
        telescopeMechanism = shoulderMechanism.append(
            new MechanismLigament2d("telescope", Constants.ArmConstants.minTranslation, 0, 5, new Color8Bit(255, 0, 0)));
        wristMechanism = telescopeMechanism.append(
            new MechanismLigament2d("wrist", 0.386, 0, 5, new Color8Bit(0, 255, 0)));

        MechanismRoot2d targetArmRoot = targetArmMechanism.getRoot("targetArm", 1.5, Constants.ArmConstants.shoulderHeight);
        targetShoulderMechanism = targetArmRoot.append(
            new MechanismLigament2d("targetShoulder", 0, -90));
        targetTelescopeMechanism = targetShoulderMechanism.append(
            new MechanismLigament2d("targetTelescope", Constants.ArmConstants.minTranslation, 0, 5, new Color8Bit(255, 0, 225)));
        targetWristMechanism = targetTelescopeMechanism.append(
            new MechanismLigament2d("targetWrist", 0.386, 0, 5, new Color8Bit(225, 255, 0)));

        shoulderAnglePot = shoulderPot.get();
        setShoulderEncoder();
    }


    @Override
    public void setArmMechanism(ArmPosition armMechanism) {
        targetPosition = armMechanism;
        // 3d renderer support
        targetShoulderMechanism.setAngle(armMechanism.shoulder);
        targetTelescopeMechanism.setLength(armMechanism.telescope);
        targetWristMechanism.setAngle(armMechanism.wrist);
    }

    @Override
    public void periodic() {
        if (!translationLimit.get()) translationLimitSwitch = true;
        if (translationLimit.get()) translationOffsetUpdated = false;
        SmartDashboard.putBoolean("translationOffsetUpdated", translationOffsetUpdated);

        shoulderAnglePot = shoulderPotFilter.calculate(shoulderPot.get());

        // Angles in radians
        currentPosition.shoulder = armJoint1.getEncoder().getPosition() * 3;
        currentPosition.wrist = wristPotFilter.calculate(wristPot.get());
        currentPosition.telescope = translationEncoderToMeters(getTranslationRelEncoder());

        shoulderMechanism.setAngle(currentPosition.shoulder);
        telescopeMechanism.setLength(currentPosition.telescope);
        wristMechanism.setAngle(currentPosition.wrist);


        SmartDashboard.putBoolean("Translational limit switch", translationLimit.get());

        Logger.getInstance().recordOutput("Temperatures/Arm 1", armJoint1.getMotorTemperature());
        Logger.getInstance().recordOutput("Temperatures/Arm 2", armJoint2.getMotorTemperature());

        SmartDashboard.putData("Arm", armMechanism);
        Logger.getInstance().recordOutput("Arm", armMechanism);
        SmartDashboard.putData("TargetArm", targetArmMechanism);
        Logger.getInstance().recordOutput("TargetArm", targetArmMechanism);
    }

    public void resetTranslation() {
        translationZeroed = false;
    }

    public void setShoulderEncoder() {
        armJoint1.getEncoder().setPosition(shoulderAnglePot / 3);
    }

    @Override
    public void execute() {
        if(RobotContainer.m_driver2Controller.getBackButtonPressed()){
            setShoulderEncoder();
        } else if(RobotContainer.m_driver2Controller.getStartButtonPressed()){
            adjustingConeGrip = true;
        }
        if(adjustingConeGrip){
            adjustConeGrip();
        }
        if(translationWait && (
            translationWaitTarget.shoulder != targetPosition.shoulder || 
            translationWaitTarget.telescope != targetPosition.telescope || 
            translationWaitTarget.wrist != targetPosition.wrist
        )){
            translationWait = false;
            translationWaitTarget = new ArmPosition(-1,-1,-1);
        }
        RobotContainer.singleton.m_fieldRelative.slowDrive = Math.abs(currentPosition.shoulder + 90) > SHOULDER_IN_BOT_RANGE;
        if(translationLimitSwitch && !translationOffsetUpdated){
            armExtend.set(ControlMode.PercentOutput, 0);
            extEncoderOffset = armExtend.getSelectedSensorPosition() - translationMetersToEncoders(0.03);
            translationZeroed = true;
            translationLimitSwitch = false;
            translationOffsetUpdated = true;
        }
        if (translationZeroed) {
            if (translationLimit.get() || translationOffsetUpdated) {
                handleTranslation();
            } else {
                SmartDashboard.putNumber("DesiredTeleMovement", 0);
                armExtend.set(ControlMode.PercentOutput, 0);
            }
            handleWrist();
            handleShoulder();
        } else {
            if (translationLimit.get()) {
                SmartDashboard.putNumber("DesiredTeleMovement", -0.3);
                armExtend.set(ControlMode.PercentOutput, -0.3);
            } else {
                SmartDashboard.putNumber("DesiredTeleMovement", 0);
                armExtend.set(ControlMode.PercentOutput, 0);
            }
        }
    }

    private void handleTranslation() {
        double extension = targetPosition.telescope;
        if (Math.abs(currentPosition.shoulder + 90) < SHOULDER_IN_BOT_RANGE) {
            extension = Math.min(extension, ArmConstants.minTranslation + 0.05);
        }
        if (translationWait && currentPosition.shoulder < -15) {
            extension = ArmConstants.minTranslation;
        }
        extension = Math.max(ArmConstants.minTranslation, Math.min(ArmConstants.maxTranslation, extension));
        double cur = currentPosition.telescope;
        if (Math.abs(cur - extension) > MAX_TELE_ERROR) {
            if (cur < extension) {
                SmartDashboard.putNumber("DesiredTeleMovement", TELE_MOVE_SPEED);
                armExtend.set(ControlMode.PercentOutput, TELE_MOVE_SPEED);
            } else if (cur > extension) {
                SmartDashboard.putNumber("DesiredTeleMovement", -TELE_MOVE_SPEED);
                armExtend.set(ControlMode.PercentOutput, -TELE_MOVE_SPEED);
            }
        } else {
            SmartDashboard.putNumber("DesiredTeleMovement", 0);
            armExtend.set(ControlMode.PercentOutput, 0);
        }
    }

    private void handleWrist() {
        double angle = targetPosition.wrist;
        if (Math.abs(currentPosition.shoulder + 90) < SHOULDER_IN_BOT_RANGE) {
            angle = 100;
        }
        angle = Math.max(Math.min(angle, 100), -70);
        double cur = currentPosition.wrist;
        double angleDiff = Math.abs(cur - angle);
        if (angleDiff > WRIST_STOP_THRESHOLD) {
            double dir = Math.signum(angle - cur);
            if (angleDiff < WRIST_SLOWDOWN_THRESHOLD) {
                SmartDashboard.putNumber("DesiredWristOutput", dir * WRIST_MOVEMENT_SPEED * (angleDiff / WRIST_SLOWDOWN_THRESHOLD));
                wrist.set(ControlMode.PercentOutput, dir * WRIST_MOVEMENT_SPEED * (angleDiff / WRIST_SLOWDOWN_THRESHOLD));
            } else {
                SmartDashboard.putNumber("DesiredWristOutput", dir * WRIST_MOVEMENT_SPEED);
                wrist.set(ControlMode.PercentOutput, dir * WRIST_MOVEMENT_SPEED);
            }
            // This is the "bang-bang" controller
            // if(cur < angle){
            //         SmartDashboard.putNumber("DesiredWristOutput", WRIST_MOVEMENT_SPEED);
            //         wrist.set(ControlMode.PercentOutput, WRIST_MOVEMENT_SPEED);
            // } else {
            //     SmartDashboard.putNumber("DesiredWristOutput", -WRIST_MOVEMENT_SPEED);
            //     wrist.set(ControlMode.PercentOutput, -WRIST_MOVEMENT_SPEED);
            // }
        } else {
            SmartDashboard.putNumber("DesiredWristOutput", 0);
            wrist.set(ControlMode.PercentOutput, 0);
        }
    }

    private void handleShoulder() {
        double degrees = targetPosition.shoulder;
        if (Math.abs(degrees + 90) > SHOULDER_IN_BOT_RANGE ||
            (Math.abs(degrees + 90) < SHOULDER_IN_BOT_RANGE &&
                currentPosition.telescope < ArmConstants.minTranslation + 0.05 &&
                Math.abs(currentPosition.wrist - 90) < WRIST_SAFE_RANGE
            )) {
            armJoint1.getPIDController().setReference(degrees / 3, CANSparkMax.ControlType.kSmartMotion);
        } else {
            armJoint1.getPIDController().setReference(currentPosition.shoulder / 3, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    private double translationEncoderToMeters(double encoder) {
        return ArmConstants.minTranslation + (encoder * (ArmConstants.maxTranslation - ArmConstants.minTranslation) / ArmConstants.maxTranslationEncoder);
    }

    private double translationMetersToEncoders(double meters) {
        // DISTANCE function
        return meters * (ArmConstants.maxTranslationEncoder / (ArmConstants.maxTranslation - ArmConstants.minTranslation));
    }

    // private double shoulderEncoderToDegrees(double encoder){
    //     return ArmConstants.shoulderMinAngle + (encoder * (ArmConstants.shoulderMaxAngle - ArmConstants.shoulderMinAngle) / (ArmConstants.shoulderMaxEncoder - ArmConstants.shoulderMinEncoder));
    // }

    private double shoulderDegreesToEncoders(double degrees) {
        return degrees * ((ArmConstants.shoulderMaxEncoder - ArmConstants.shoulderMinEncoder) / (ArmConstants.shoulderMaxAngle - ArmConstants.shoulderMinAngle));
    }

    private double getTranslationRelEncoder() {
        return -(armExtend.getSelectedSensorPosition() - extEncoderOffset);
    }

    public boolean shoulderInPlace() {
        return (Math.abs(shoulderDegreesToEncoders(targetPosition.shoulder) - armJoint1.getEncoder().getPosition()) < 10);
    }

    @Override
    public boolean armInPlace() {
        SmartDashboard.putNumber("ShoulderError", Math.abs(targetPosition.shoulder - currentPosition.shoulder));
        SmartDashboard.putNumber("TeleError", Math.abs(targetPosition.telescope - currentPosition.telescope));
        SmartDashboard.putNumber("WristError", Math.abs(targetPosition.wrist - currentPosition.wrist));

        return (Math.abs(targetPosition.shoulder - currentPosition.shoulder) < 5 && Math.abs(targetPosition.telescope - currentPosition.telescope) < 0.04 && Math.abs(targetPosition.wrist - currentPosition.wrist) < 8);
    }

    private boolean adjustingConeGrip = false;
    private boolean startedAdjustingGrip = false;
    private boolean reachedLowTarget = false;
    private Timer coneGripTimer = new Timer();

    private void adjustConeGrip(){
        if(!startedAdjustingGrip){
            RobotContainer.singleton.m_claw.toggleClaw(false);
            targetPosition.telescope = ArmConstants.minTranslation + 0.05;
            startedAdjustingGrip = true;
        }
        if(!reachedLowTarget && Math.abs(targetPosition.telescope - currentPosition.telescope) < 0.008){
            coneGripTimer.reset();
            coneGripTimer.start();
            reachedLowTarget = true;
        }
        if(reachedLowTarget && coneGripTimer.get() > 0.25){
            Robot.singleton.manualRollers = false;
            RobotContainer.singleton.m_claw.toggleClaw(true);
            RobotContainer.singleton.m_claw.setRollers(0.35);
        }
        if(reachedLowTarget && coneGripTimer.get() > 0.9){
            Robot.singleton.manualRollers = true;
            RobotContainer.singleton.m_claw.setRollers(0);
            targetPosition.telescope = ArmConstants.minTranslation;
            coneGripTimer.stop();
            adjustingConeGrip = false;
            reachedLowTarget = false;
            startedAdjustingGrip = false;
        }
    }

    @Override
    public void enableTranslationWait(){
        translationWait = true;
        translationWaitTarget = new ArmPosition(targetPosition.shoulder, targetPosition.telescope, targetPosition.wrist);
    }
}
