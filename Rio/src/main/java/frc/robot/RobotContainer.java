package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.supers.*;
import frc.robot.subsystems_sim.*;
import frc.robot.subsystems.*;

import frc.robot.commands.*;
import frc.robot.commands.Drive.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.packages.pathfinding.Pathfinder;
import frc.packages.pathfinding.PathfindingWS;

import java.io.FileNotFoundException;
import java.net.URI;

import org.json.JSONObject;
import frc.packages.util.ConfigLoader;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.PneumaticHub;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static RobotContainer singleton;

    public static PneumaticHub m_pH;
    // Scheduler
    public HighLevelScheduler m_scheduler;

    // Subsystems
    public HDriveSuper m_robotDrive;
    public MayTagsSim m_juneTag;
    public ArmSuper m_arm;
    public ClawSuper m_claw;
    public LowLevelVelocitySuper m_lowLevelVelocity;
    public LowLevelRotation m_lowLevelRotation;

    // Commands
    public FollowTrajectory m_followTrajectory;
    public ArcadeDrive m_arcadeDrive;
    public FieldRelative m_fieldRelative;
    public PointController m_pointController;
    public StartPathfindCommand m_startPathfind;
    public PickupCubeCommand m_pickupCubeCommand;
    public ArmCommand m_armCommand;

    // Pathfinding System
    public final Pathfinder m_pathfinder;

    public final PathfindingWS m_pathfinderWS;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Controllers
    public static CommandXboxController m_driverController1Command = new CommandXboxController(
        Constants.OIConstants.kDriver1ControllerPort);
    public static CommandXboxController m_driverController2Command = new CommandXboxController(
        Constants.OIConstants.kDriver2ControllerPort);
    public static XboxController m_driver1Controller = new XboxController(
        Constants.OIConstants.kDriver1ControllerPort);
    public static XboxController m_driver2Controller = new XboxController(
        Constants.OIConstants.kDriver2ControllerPort);

    // PID Controllers
    public SparkMaxPIDController leftController;
    public SparkMaxPIDController rightController;
    public SparkMaxPIDController lateralController;
    public PIDController rotationPIDController = new PIDController(18, 0, 1);
    public PIDController pointPIDController = new PIDController(3.1, 0, 0.0018);
    public PIDController pathAccuracyController = new PIDController(0, 0, 0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws FileNotFoundException {
        // Config Handling
        ConfigLoader.loadConfig();

        // PID Configs
        JSONObject leftPID = ConfigLoader.pidMotors.getJSONObject("left");
        JSONObject rightPID = ConfigLoader.pidMotors.getJSONObject("right");
        JSONObject lateralPID = ConfigLoader.pidMotors.getJSONObject("lateral");

        // General Assignments
        m_pathfinder = new Pathfinder();

        // Sim & Real Assignments
        if (Robot.isSimulation()) {
            m_robotDrive = new HDriveSim();
            m_juneTag = new MayTagsSim();
            m_arm = new ArmSim();
            m_claw = new ClawSim();
            m_lowLevelVelocity = new LowLevelVelocitySim(
                HDriveSuper.poseEstimator::getEstimatedPosition,
                DriveConstants.kDriveFeedforward,
                DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(leftPID.getDouble("p"), leftPID.getDouble("i"), leftPID.getDouble("d")),
                new PIDController(rightPID.getDouble("p"), rightPID.getDouble("i"), rightPID.getDouble("d")),
                new PIDController(lateralPID.getDouble("p"), lateralPID.getDouble("i"), lateralPID.getDouble("d")),
                m_robotDrive::tankDriveVolts);
            m_lowLevelRotation = new LowLevelRotation(rotationPIDController,
                HDriveSuper.poseEstimator::getEstimatedPosition,
                m_lowLevelVelocity);

            m_pathfinderWS = new PathfindingWS(URI.create("ws://localhost:5800"), m_pathfinder);
            m_pathfinderWS.connect();
        } else if (Robot.isReal()) {
            m_pH = new PneumaticHub(3);
            m_robotDrive = new HDrive();
            m_arm = new Arm();
            m_claw = new Claw();
            leftController = m_robotDrive.leftMotor1.getPIDController();
            rightController = m_robotDrive.rightMotor1.getPIDController();
            lateralController = m_robotDrive.lateralMotor1.getPIDController();
            m_lowLevelVelocity = new LowLevelVelocity(
                HDriveSuper.poseEstimator::getEstimatedPosition,
                DriveConstants.kDriveKinematics,
                leftController,
                rightController,
                lateralController
            );
            m_lowLevelRotation = new LowLevelRotation(rotationPIDController,
                HDriveSuper.poseEstimator::getEstimatedPosition,
                m_lowLevelVelocity);

            m_pathfinderWS = new PathfindingWS(URI.create("ws://10.30.44.8:5800"), m_pathfinder);
            m_pathfinderWS.connect();
        } else {
            m_pathfinderWS = null;
            System.out.println("WARNING: Not Real or Sim");
        }

        declareCommands();
        configureButtonBindings();

        singleton = this;
    }

    private void declareCommands() {
        m_lowLevelRotation = new LowLevelRotation(
            rotationPIDController,
            HDriveSuper.poseEstimator::getEstimatedPosition,
            m_lowLevelVelocity);
        m_pointController = new PointController(
            m_lowLevelVelocity,
            m_lowLevelRotation,
            pointPIDController,
            HDriveSuper.poseEstimator::getEstimatedPosition, m_robotDrive);
        m_followTrajectory = new FollowTrajectory(m_robotDrive, m_pathfinderWS, m_lowLevelVelocity, m_lowLevelRotation);
        m_startPathfind = new StartPathfindCommand(m_followTrajectory, m_pointController);
        m_fieldRelative = new FieldRelative(m_lowLevelVelocity, m_lowLevelRotation, m_driver1Controller, m_robotDrive);
        m_arcadeDrive = new ArcadeDrive(m_robotDrive, m_driver1Controller);
        m_pickupCubeCommand = new PickupCubeCommand(m_startPathfind, m_pathfinderWS);
        m_scheduler = new HighLevelScheduler(m_fieldRelative, m_arcadeDrive);
        m_armCommand = new ArmCommand(m_arm);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Trigger bButton = new JoystickButton(m_driver1Controller, 2);
        bButton.onTrue(m_arcadeDrive);
        Trigger rightBumper = new JoystickButton(m_driver1Controller, 6);
        rightBumper.onTrue(m_fieldRelative);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // TODO swap this over to scheduleAutonomous system using high level scheduler
    public Command getAutonomousCommand() {
        m_pickupCubeCommand.setCubeCenter(new Pose3d(11.75, 2.8, 0.1, new Rotation3d(0, 0, 0)));
        return autoChooser.get();
    }

    public HDriveSuper getRobotDrive() {
        return m_robotDrive;
    }
}
