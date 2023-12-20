package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.packages.HDrive.HDriveKinematics;
import frc.packages.HDrive.HDrivetrainLateralSystemId;
import frc.packages.InverseKinematicsRER.ArmPosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class AdvantageKitConstants {
        public static final Mode currentMode = Mode.REAL;

        public enum Mode {
            /**
             * Running on a real robot.
             */
            REAL,

            /**
             * Running a physics simulator.
             */
            SIM,

            /**
             * Replaying from a log file.
             */
            REPLAY
        }
    }

    public static class GearboxConstants {
        public final static double kDriveRatio = 8.45;
        public final static double kLateralRatio = 10.71;
        public final static double kClawRatio = 15;
        public final static double kArmTeleSlideRatio = 93.75;
        public final static double kArmRatio = 120;
        public final static double kWristRatio = 328.13;
    }

    public static class ArmConstants {
        // Measurements
        public final static double minTranslation = 0.57785;
        public final static double maxTranslation = 1.14475;
        public final static double maxTranslationEncoder = 16868;
        public final static double shoulderHeight = 0.981964;
        public final static double maxShoulderAngle = 15;


        public final static double wristMinAngle = -90;
        public final static double wristMaxAngle = 90;
        public final static double wristMinEncoder = 0;
        public final static double wristMaxEncoder = 3770;

        public final static double shoulderMinAngle = -90;
        public final static double shoulderMaxAngle = 90;
        public final static double shoulderMinEncoder = 0;
        public final static double shoulderMaxEncoder = 60;

        // TODO: Tune this
        public final static double wristToGamePiece = 0.386;
    }

    public static final class ArmPositions {
        public static final ArmPosition scoringHighCubeFront = new ArmPosition(13, 0.7, 15);
        public static final ArmPosition scoringHighConeFront = new ArmPosition(20, 1.1, 21);
        public static final ArmPosition scoringMidCubeFront = new ArmPosition(-30, ArmConstants.minTranslation, 80);
        public static final ArmPosition scoringMidConeFront = new ArmPosition(-16, 0.8, 90);
        public static final ArmPosition shelfFront = new ArmPosition(13, ArmConstants.minTranslation, 15);
        public static final ArmPosition groundFront = new ArmPosition(-46, 1.02, 60);
        public static final ArmPosition idleFront = new ArmPosition(-90, ArmConstants.minTranslation, 90);
    }

    public static class VisionConstants {
        public final static double kDT = 0.01;
        //robot to cam
        // +x is forward
        // +y is left
        public final static Transform3d robotToCam1 = //front camera
            new Transform3d(
                new Translation3d(
                    0.31115,
                    -0.2413,
                    0.4953),
                new Rotation3d(
                    0,
                    0,
                    0.017));
        public final static Transform3d robotToCam2 = //back camera 
            new Transform3d(
                new Translation3d(
                    -0.31115,
                    -0.2413,
                    0.5207),
                new Rotation3d(
                    0,
                    0,
                    3.14));
        public final static Transform3d robotToCam3 = //back camera
            new Transform3d(
                new Translation3d(
                    0,
                    0,
                    0
                ),
                new Rotation3d(
                    0,
                    0,
                    0
                )
            );
        public final static Transform3d robotToCam4 = //back camera
            new Transform3d(
                new Translation3d(
                    0,
                    0,
                    0
                ),
                new Rotation3d(
                    0,
                    0,
                    0
                )
            );
        // from center.
        public static final String camera1Name = "front";
        public static final String camera2Name = "back";
        // public static final String camera3Name = "left";
        // public static final String camera4Name = "right";
    }

    public static final class DriveConstants {
        public static final double kMaxAcceleration = 0.15;
        public static final double kMaxLateralAcceleration = 0.1;
        public static final int kLeftMotor1Port = 13;
        public static final int kLeftMotor2Port = 14;
        public static final int kRightMotor1Port = 11;
        public static final int kRightMotor2Port = 12;
        public static final int kLateralMotor1Port = 15;
        public static final int kLateralMotor2Port = 16;

        public static final int kForwardLateral1Port = 1;
        public static final int kReverseLateral1Port = 0;
        public static final int kForwardLateral2Port = 2;
        public static final int kReverseLateral2Port = 3;

        public static final int kShoulderPort1 = 21;
        public static final int kShoulderPort2 = 22;
        public static final int kTelescopePort = 23;
        public static final int kWristPort = 33;

        public static final int kRightRollersPort = 31;
        public static final int kLeftRollersPort = 32;
        public static final int kForwardClawPort = 4;
        public static final int kReverseClawPort = 6;


        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final int[] kLateralEncoderPorts = new int[]{4, 5};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
        public static final boolean kLateralEncoderReversed = false;

        public static final double kTrackwidthMeters = 0.69;
        public static final HDriveKinematics kDriveKinematics = new HDriveKinematics(
            kTrackwidthMeters);

        public static final int kSimEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kSimEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / kSimEncoderCPR;
        /* TODO Check if these are needed or if we can pull this data straight from the
        CANSparkMax devices.
        */
        public static final double kEncoderCPR = 40;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // TODO RETUNE THESE USING SYSID
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;

        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
        public static final double kDriveGearing = 10.71;

        public static double kMassKG = 60;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

        public static final LinearSystem<N1, N1, N1> kLateralDrivetrainPlant =

            HDrivetrainLateralSystemId.identifyLateralVelocitySystem(
                kvVoltSecondsPerMeter / 2,
                kaVoltSecondsSquaredPerMeter / 2,
                kvVoltSecondsPerRadian / 2,
                kaVoltSecondsSquaredPerRadian / 2);

        public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(
            ksVolts,
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter);

        public static final double leftWheelRadius = 0.1524;
        public static final double rightWheelRadius = 0.1524;
        public static final double lateralWheelRadius = 0.1016;
    }

    public static final class OIConstants {
        public static final int kDriver1ControllerPort = 0;
        public static final int kDriver2ControllerPort = 1;
        public static final double kDriverDeadband = 0.1;
        public static final double kDriverRotationDeadband = 0.45;
        public static final double kRotationDeadband = 0.2;
    }
}
