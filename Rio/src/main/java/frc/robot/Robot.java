package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.packages.InverseKinematicsRER.ArmPosition;
import frc.packages.pathfinding.Structures.Vertex;
import frc.packages.util.ConfigLoader;
import frc.robot.subsystems_sim.HDriveSim;

import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.supers.ArmSuper;
import frc.robot.supers.HDriveSuper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.FileInputStream;
import java.io.FileNotFoundException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot/LoggedRobot
 * documentation. If you guys need to mess with this let me know I'll change the VM
 * & gradle configurations ~ Autumn O.
 */
public class Robot extends LoggedRobot {
    private RobotContainer m_robotContainer;

    private InControlInterface m_Interface;

    private Command m_autonomousCommand;

    private final Timer wsReconnectTimer = new Timer();

    private boolean armEnabled = true;

    public boolean manualRollers = true;

    public static Robot singleton;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // UsbCamera clawFeed = CameraServer.startAutomaticCapture();
        CameraServer.startAutomaticCapture(0);
        // clawFeed.setFPS(10);
        //clawFeed.setResolution(0, 0)

        singleton = this;
        Logger logger = Logger.getInstance();

        // Record metadata
        logger.recordMetadata("ProjectName", Metadata.MAVEN_NAME);
        logger.recordMetadata("BuildDate", Metadata.BUILD_DATE);
        logger.recordMetadata("GitSHA", Metadata.GIT_SHA);
        logger.recordMetadata("GitDate", Metadata.GIT_DATE);
        logger.recordMetadata("GitBranch", Metadata.GIT_BRANCH);
        switch (Metadata.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (isReal()) {
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            m_Interface = new InControlInterface();
            m_Interface.setup();
            wsReconnectTimer.start();
        } else if (isSimulation()) {
            logger.addDataReceiver(new WPILOGWriter("logs/"));
            logger.addDataReceiver(new NT4Publisher());
            m_Interface = new InControlInterface();
            m_Interface.setup();
            wsReconnectTimer.start();
        }

        // Start AdvantageKit logging framework
        logger.start();

        try {
            m_robotContainer = new RobotContainer();
            // TODO this needs a better way
            ConfigLoader.configurePID();
        } catch (FileNotFoundException e) {
            // Drive Station error reporting
            DriverStation.reportError("Fatal: Missing field map", e.getStackTrace());

            // Console reporting
            System.out.println("Fatal: Missing field map");
            e.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * This function is called periodically during all modes.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putNumber("Debug/Arm/Wrist Pot", Arm.wristPot.get());
        SmartDashboard.putNumber("Debug/Arm/Shoulder Pot", Arm.shoulderPot.get());

        // Log robot pose estimate, note rotation in radians
        // TODO find a better spot for this
        Logger.getInstance().recordOutput("Robot Pose Estimate", HDriveSuper.poseEstimator.getEstimatedPosition());
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // hard kill drive velocity system
        RobotContainer.singleton.m_lowLevelVelocity.end();
        RobotContainer.singleton.m_lowLevelVelocity.setEnabled(false);
        m_robotContainer.m_robotDrive.tankDriveVolts(0, 0, 0);
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    // TODO swap this to scheduleAutonomous using HL scheduler
    @Override
    public void autonomousInit() {
        m_robotContainer.m_arm.setArmMechanism(Constants.ArmPositions.scoringHighCubeFront);
        m_robotContainer.m_arm.enableTranslationWait();
        // TODO pop arm zeroing here as a proper system
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        RobotContainer.singleton.m_lowLevelVelocity.setEnabled(true);
        if(DriverStation.getAlliance() == Alliance.Blue) RobotContainer.singleton.m_lowLevelRotation.setTargetRotation(Rotation2d.fromDegrees(180));
    }

    private boolean startedRollers = false;
    private boolean armAutoIsDone = false;
    private boolean startedPointController = false;
    private boolean done = false;
    private Timer autoRollerTimer = new Timer();

    @Override
    public void autonomousPeriodic() {
        RobotContainer.singleton.m_lowLevelVelocity.execute();
        RobotContainer.singleton.m_lowLevelRotation.execute();
        // TODO commandify & put into an auto chooser
        ArmSuper arm = m_robotContainer.m_arm;
        if (armEnabled) arm.execute();
        boolean isRed = DriverStation.getAlliance() == Alliance.Red;
        if(done) return;
        if(startedPointController){
            double x = HDriveSuper.poseEstimator.getEstimatedPosition().getX();
            if(isRed && x < 11){
                RobotContainer.singleton.m_lowLevelRotation.setTargetRotation(Rotation2d.fromDegrees(180));
                arm.setArmMechanism(Constants.ArmPositions.groundFront);
            } else if (!isRed && x > 5.5){
                RobotContainer.singleton.m_lowLevelRotation.setTargetRotation(Rotation2d.fromDegrees(0));
                arm.setArmMechanism(Constants.ArmPositions.groundFront);
            }
            done = true;
            return;
        }
        if (armAutoIsDone && autoRollerTimer.get() > 2.2){
            autoRollerTimer.stop();
            int location = 2;
            
            double yValue = HDriveSuper.poseEstimator.getEstimatedPosition().getY();
            if(yValue != 0){
                if(yValue > 3.8){
                    location = isRed ? 3 : 1;
                } else if(yValue < 1.7) {
                    location = isRed ? 1 : 3;
                    m_robotContainer.m_robotDrive.engageLateralDrive(false);
                }
            }
            if(location == 2){
                done = true;
                return;
            }
            if(isRed){
                if(location == 1){
                    RobotContainer.singleton.m_pointController.setTarget(new Vertex(10, 0.78));
                    RobotContainer.singleton.m_pointController.schedule();
                } else if(location == 3){
                    RobotContainer.singleton.m_pointController.setTarget(new Vertex(10, 5.15));
                    RobotContainer.singleton.m_pointController.schedule();
                };
            } else {
                if(location == 1){
                    RobotContainer.singleton.m_pointController.setTarget(new Vertex(6.5,5.15));
                    RobotContainer.singleton.m_pointController.schedule();
                } else if(location == 3){
                    RobotContainer.singleton.m_pointController.setTarget(new Vertex(6.5,0.78));
                    RobotContainer.singleton.m_pointController.schedule();
                };
            }
            startedPointController = true;
        }
        if (arm.armInPlace() && !startedRollers) {
            m_robotContainer.m_claw.setRollers(-0.7);
            autoRollerTimer.reset();
            autoRollerTimer.start();
            startedRollers = true;
        }
        if (startedRollers && autoRollerTimer.get() > 1) {
            m_robotContainer.m_claw.setRollers(0);
            m_robotContainer.m_arm.setArmMechanism(Constants.ArmPositions.idleFront);
            armAutoIsDone = true;
        }
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        m_robotContainer.m_arm.setArmMechanism(Constants.ArmPositions.idleFront);
        m_robotContainer.m_fieldRelative.schedule();
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.cancel();
        // }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        RobotContainer.singleton.m_lowLevelVelocity.execute();
        RobotContainer.singleton.m_lowLevelRotation.execute();

        if (RobotContainer.m_driver1Controller.getYButton()) {
            ConfigLoader.configurePID();
        }

        ArmSuper arm = m_robotContainer.m_arm;
        if (armEnabled) arm.execute();
        //RobotContainer.singleton.m_armCommand.execute();

        m_Interface.checkForUpdates();
        if(Robot.isSimulation()){
            // Scheduler System
            m_robotContainer.m_scheduler.execute();
        }

        // Arm testing code
        double leftStick = -RobotContainer.m_driver2Controller.getLeftY();
        if (Math.abs(leftStick) < 0.1) {
            leftStick = 0;
        }
        double rightStick = -RobotContainer.m_driver2Controller.getRightY();
        if (Math.abs(rightStick) < 0.1) {
            rightStick = 0;
        }

        if (RobotContainer.m_driver2Controller.getXButtonPressed()) {
            arm.setArmMechanism(Constants.ArmPositions.idleFront);
        } else if (RobotContainer.m_driver2Controller.getLeftTriggerAxis() < 0.8 && RobotContainer.m_driver2Controller.getAButtonPressed()) {
            arm.setArmMechanism(Constants.ArmPositions.shelfFront);
        } else if (RobotContainer.m_driver2Controller.getLeftTriggerAxis() > 0.8) {
            if (RobotContainer.m_driver2Controller.getBButtonPressed()) {
                arm.setArmMechanism(Constants.ArmPositions.scoringMidCubeFront);
            } else if (RobotContainer.m_driver2Controller.getYButtonPressed()) {
                arm.setArmMechanism(Constants.ArmPositions.scoringHighCubeFront);
                arm.enableTranslationWait();
            } else if (RobotContainer.m_driver2Controller.getAButtonPressed()) {
                arm.setArmMechanism(Constants.ArmPositions.groundFront);
            }
        } else if (RobotContainer.m_driver2Controller.getRightTriggerAxis() > 0.8) {
            if (RobotContainer.m_driver2Controller.getBButtonPressed()) {
                arm.setArmMechanism(Constants.ArmPositions.scoringMidConeFront);
            } else if (RobotContainer.m_driver2Controller.getYButtonPressed()) {
                arm.setArmMechanism(Constants.ArmPositions.scoringHighConeFront);
                arm.enableTranslationWait();
            }
        }
        ArmPosition targetPosition = arm.targetPosition;
        ArmPosition newTarget = new ArmPosition(targetPosition.shoulder, targetPosition.telescope, targetPosition.wrist);
        newTarget.shoulder = targetPosition.shoulder + (leftStick * 0.4);
        if (!RobotContainer.m_driver2Controller.getLeftBumper())
            newTarget.wrist = targetPosition.wrist + (rightStick * 1);
        else
            newTarget.telescope = Math.min(Math.max(targetPosition.telescope + (rightStick * 0.006), ArmConstants.minTranslation), ArmConstants.maxTranslation);

        arm.setArmMechanism(newTarget);


        double POV = RobotContainer.m_driver2Controller.getPOV();
        if(manualRollers){
            if (POV == 0) {
                m_robotContainer.m_claw.setRollers(RobotContainer.m_driver2Controller.getLeftTriggerAxis() > 0.8 ? -0.65 : -1);
            } else if (POV == 180) {
                m_robotContainer.m_claw.setRollers(0.6);
            } else {
                m_robotContainer.m_claw.setRollers(0);
            }
        }

        if (RobotContainer.m_driver2Controller.getRightBumperPressed()) {
            m_robotContainer.m_claw.toggleClaw(RobotContainer.m_driver2Controller.getLeftTriggerAxis() > 0.8 ? true: false);
        }
        // Toggle Lateral
        if (RobotContainer.m_driver1Controller.getLeftBumperPressed()) {
            m_robotContainer.m_robotDrive.toggleLateralDrive();
        }
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.>
        CommandScheduler.getInstance().cancelAll();

        // TODO setup proper testing infrastructure no pile of if statements
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        // TODO setup proper testing infrastructure no pile of if statements
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
        // TODO form better way to trigger this
        m_robotContainer.m_robotDrive.resetOdometry(new Pose2d(2, 2.5, new Rotation2d(0)));
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // TODO schedule this properly
        m_robotContainer.m_juneTag.simulationPeriodic();

        // Battery simulation
        double drawCurrent = HDriveSim.drivetrainSimulator.getCurrentDrawAmps();
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
        SmartDashboard.putNumber("Sim Power System/Bat. Volt.", loadedVoltage);
        SmartDashboard.putNumber("Sim Power System/Current Draw", drawCurrent);
        RoboRioSim.setVInVoltage(loadedVoltage);
    }
}
