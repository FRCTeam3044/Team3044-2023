package frc.packages.util;

import java.io.FileInputStream;
import java.io.FileNotFoundException;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Filesystem;

public class ConfigLoader {
    public static JSONObject pidMotors;
    public static JSONObject pidHighLevel;
    public static JSONObject arm;
    public static JSONObject drive;
    public static JSONObject pid;

    public static JSONArray redScoringNodes;
    public static JSONArray blueScoringNodes;
    public static double scoringNodeOffset = 0.0;

    public static void loadConfig() {
        loadPIDVals();
        try {
            FileInputStream inputNodes = new FileInputStream(Filesystem.getDeployDirectory() + "/nodes.json");
            JSONTokener nodeTokener = new JSONTokener(inputNodes);
            JSONObject nodes = new JSONObject(nodeTokener);
            redScoringNodes = nodes.getJSONArray("red");
            blueScoringNodes = nodes.getJSONArray("blue");
            scoringNodeOffset = nodes.getDouble("offset");
        } catch (FileNotFoundException e) {
            System.out.println("Scoring node data not found");
            System.exit(1);
        }
    }

    private static void loadPIDVals() {
        try {
            FileInputStream inputPid = new FileInputStream(Filesystem.getDeployDirectory() + "/pid.json");
            JSONTokener pidTokener = new JSONTokener(inputPid);
            pid = new JSONObject(pidTokener);
            pidMotors = pid.getJSONObject("motors");
            arm = pid.getJSONObject("arm");
            drive = pid.getJSONObject("drive");
            if (Robot.isReal()) {
                pidHighLevel = pid.getJSONObject("high-level");
            } else {
                pidHighLevel = pid.getJSONObject("high-level-sim");
            }
        } catch (FileNotFoundException e) {
            System.out.println("PID data not found");
            System.exit(1);
        }
    }

    public static void configurePID() {
        loadPIDVals();
        // Read from config
        JSONObject leftPID = ConfigLoader.pidMotors.getJSONObject("left");
        JSONObject rightPID = ConfigLoader.pidMotors.getJSONObject("right");
        JSONObject lateralPID = ConfigLoader.pidMotors.getJSONObject("lateral");
        JSONObject rotPID = ConfigLoader.pidHighLevel.getJSONObject("rotation");
        JSONObject pointPID = ConfigLoader.pidHighLevel.getJSONObject("point");
        JSONObject pathAccuracyPID = ConfigLoader.pidHighLevel.getJSONObject("path-accuracy");
        JSONObject shoulderValues = ConfigLoader.arm.getJSONObject("shoulder");
        JSONObject wristValues = ConfigLoader.arm.getJSONObject("wrist");
        JSONObject telescopeValues = ConfigLoader.arm.getJSONObject("telescope");
        // Caching the robot container for readability
        RobotContainer RC = RobotContainer.singleton;

        if (Robot.isReal()) {
            // Drive Motors
            RC.leftController.setP(leftPID.getDouble("p"));
            RC.leftController.setI(leftPID.getDouble("i"));
            RC.leftController.setD(leftPID.getDouble("d"));
            RC.leftController.setIZone(leftPID.getDouble("iz"));
            RC.leftController.setFF(leftPID.getDouble("ff"));
            RC.leftController.setOutputRange(leftPID.getDouble("minOutput"), leftPID.getDouble("maxOutput"));
            RC.rightController.setP(rightPID.getDouble("p"));
            RC.rightController.setI(rightPID.getDouble("i"));
            RC.rightController.setD(rightPID.getDouble("d"));
            RC.rightController.setIZone(rightPID.getDouble("iz"));
            RC.rightController.setFF(rightPID.getDouble("ff"));
            RC.rightController.setOutputRange(rightPID.getDouble("minOutput"), rightPID.getDouble("maxOutput"));
            RC.lateralController.setP(lateralPID.getDouble("p"));
            RC.lateralController.setI(lateralPID.getDouble("i"));
            RC.lateralController.setD(lateralPID.getDouble("d"));
            RC.lateralController.setIZone(lateralPID.getDouble("iz"));
            RC.lateralController.setFF(lateralPID.getDouble("ff"));
            RC.lateralController.setOutputRange(lateralPID.getDouble("minOutput"), lateralPID.getDouble("maxOutput"));
        }

        // Rotation controller
        RC.rotationPIDController.setP(rotPID.getDouble("maxSpeed") / Math.toRadians(rotPID.getDouble("maxSpeedAngleThreshold")));
        RC.m_lowLevelRotation.minSpeed = rotPID.getDouble("minSpeed");
        RC.m_lowLevelRotation.maxSpeed = rotPID.getDouble("maxSpeed");
        // Point controller
        RC.pointPIDController.setP(pointPID.getDouble("maxSpeed") / pointPID.getDouble("maxSpeedDistanceThreshold"));
        RC.m_pointController.minSpeed = pointPID.getDouble("minSpeed");
        RC.m_pointController.maxSpeed = pointPID.getDouble("maxSpeed");
        RC.m_pointController.threshold = pointPID.getDouble("threshold");
        RC.m_pointController.minTime = pointPID.getDouble("minTime");
        // Path accuracy controller
        RC.pathAccuracyController.setP(pathAccuracyPID.getDouble("p"));
        RC.pathAccuracyController.setI(pathAccuracyPID.getDouble("i"));
        RC.pathAccuracyController.setD(pathAccuracyPID.getDouble("d"));

        // Arm
        if(RC.m_arm instanceof Arm){
            SparkMaxPIDController armPID = ((Arm)RC.m_arm).armJoint1.getPIDController();
            armPID.setSmartMotionMaxVelocity(shoulderValues.getDouble("maxVel"), 0);
            armPID.setSmartMotionMaxAccel(shoulderValues.getDouble("maxAccel"), 0);
            armPID.setSmartMotionAllowedClosedLoopError(shoulderValues.getDouble("minError"), 0);
        }
        RC.m_arm.SHOULDER_IN_BOT_RANGE = shoulderValues.getDouble("inBotRange");

        RC.m_arm.WRIST_MOVEMENT_SPEED = wristValues.getDouble("speed");
        RC.m_arm.WRIST_SLOWDOWN_THRESHOLD = wristValues.getDouble("slowdown");
        RC.m_arm.WRIST_STOP_THRESHOLD = wristValues.getDouble("stop");

        RC.m_arm.MAX_TELE_ERROR = telescopeValues.getDouble("deadband");
        RC.m_arm.TELE_MOVE_SPEED = telescopeValues.getDouble("speed");

        RC.m_fieldRelative.SPEED = drive.getDouble("speed");
        RC.m_fieldRelative.SLOW_SPEED = drive.getDouble("armOutSpeed");
    }
}
