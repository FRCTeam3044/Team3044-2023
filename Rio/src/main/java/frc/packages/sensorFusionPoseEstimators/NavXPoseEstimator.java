package frc.packages.sensorFusionPoseEstimators;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.WPIUtilJNI;

import java.util.Map;
import java.util.Objects;

public class NavXPoseEstimator {
    Pose2d m_estimatedPosition;
    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());

    private final TimeInterpolatableBuffer<InterpolationRecord> m_poseBuffer =
        TimeInterpolatableBuffer.createBuffer(1.5);

    /**
     * Constructs a {@link NavXPoseEstimator} with default standard deviations for the model and
     * vision measurements.
     *
     * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for
     * y, and 0.02 radians for heading. The default standard deviations of the vision measurements are
     * 0.05 meters for x, 0.05 meters for y, and 0.1 radians for heading.
     *
     * @param navXYaw           The current gyro angle determined by your navX
     * @param globalXMeters     the total distance on X axis traveled mesured by NavX global velocity
     * @param globalYMeters     the total distance on Y axis traveled mesured by NavX global velocity
     * @param initialPoseMeters The starting pose estimate.
     */
    public NavXPoseEstimator(
        Rotation2d navXYaw,
        double globalXMeters,
        double globalYMeters,
        Pose2d initialPoseMeters) {
        this(
            navXYaw,
            globalXMeters,
            globalYMeters,
            initialPoseMeters,
            VecBuilder.fill(0.1, 0.1, 0.02),
            VecBuilder.fill(0.05, 0.05, 0.1));
    }

    /**
     * Constructs a {@link NavXPoseEstimator} with default standard deviations for the model and
     * vision measurements.
     *
     * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for
     * y, and 0.02 radians for heading. The default standard deviations of the vision measurements are
     * 0.05 meters for x, 0.05 meters for y, and 0.1 radians for heading.
     *
     * @param navXYaw                  The current gyro angle determined by your navX
     * @param globalXMeters            the total distance on X axis traveled mesured by NavX global velocity
     * @param globalYMeters            the total distance on Y axis traveled mesured by NavX global velocity
     * @param initialPoseMeters        The starting pose estimate.
     * @param stateStdDevs             Standard deviations of the pose estimate (x position in meters, y position
     *                                 in meters, and heading in radians). Increase these numbers to trust your state estimate
     *                                 less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *                                 in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public NavXPoseEstimator(
        Rotation2d navXYaw,
        double globalXMeters,
        double globalYMeters,
        Pose2d initialPoseMeters,
        Matrix<N3, N1> stateStdDevs,
        Matrix<N3, N1> visionMeasurementStdDevs) {

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }

        // Initialize vision R
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     *                                 numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *                                 theta]ᵀ, with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                    row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
            }
        }
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose your robots current pose
     */
    public void resetPosition(
        Pose2d pose) {
        // Reset state estimate and error covariance
        m_estimatedPosition = pose;
        m_poseBuffer.clear();
    }

    /**
     * Gets the estimated robot pose.
     *
     * @return The estimated robot pose in meters.
     */
    public Pose2d getEstimatedPosition() {
        return m_estimatedPosition;
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * NavXPoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note that if you
     *                              don't use your own time source by calling {@link
     *                              NavXPoseEstimator#updateWithTime(double, Rotation2d, double, double)} then you
     *                              must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is
     *                              the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.) This means that
     *                              you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source
     *                              or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Step 1: Get the pose odometry measured at the moment the vision measurement was made.
        var sample = m_poseBuffer.getSample(timestampSeconds);

        if (sample.isEmpty()) {
            return;
        }

        // Step 1: Reset Odometry to state at sample with vision adjustment.
        m_estimatedPosition = sample.get().poseMeters;

        // Step 2: Replay odometry inputs between sample time and latest recorded sample to update the
        // pose buffer and correct odometry.
        for (Map.Entry<Double, InterpolationRecord> entry :
            m_poseBuffer.getInternalBuffer().tailMap(timestampSeconds).entrySet()) {
            updateWithTime(
                entry.getKey(),
                entry.getValue().navXYaw,
                entry.getValue().globalXMeters,
                entry.getValue().globalYMeters);
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * NavXPoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to {@link
     * NavXPoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in seconds. Note that if you
     *                                 don't use your own time source by calling {@link
     *                                 NavXPoseEstimator#updateWithTime(double, Rotation2d, double, double)}, then you
     *                                 must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is
     *                                 the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that
     *                                 you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source
     *                                 in this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *                                 in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param navXYaw       The current gyro angle.
     * @param globalXMeters The total distance travelled by the left wheel in meters.
     * @param globalYMeters The total distance travelled by the right wheel in meters.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d update(
        Rotation2d navXYaw, double globalXMeters, double globalYMeters) {
        return updateWithTime(
            WPIUtilJNI.now() * 1.0e-6, navXYaw, globalXMeters, globalYMeters);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param navXYaw            The current gyro angle.
     * @param globalXMeters      The total distance travelled by the NavX in meters.
     * @param globalYMeters      The total distance travelled by the NavX in meters.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(
        double currentTimeSeconds,
        Rotation2d navXYaw,
        double globalXMeters,
        double globalYMeters) {
        m_estimatedPosition = new Pose2d(globalXMeters, globalYMeters, navXYaw);
        m_poseBuffer.addSample(
            currentTimeSeconds,
            new InterpolationRecord(
                getEstimatedPosition(), navXYaw, globalXMeters, globalYMeters));

        return getEstimatedPosition();
    }

    /**
     * Represents an odometry record. The record contains the inputs provided as well as the pose that
     * was observed based on these inputs, as well as the previous record and its inputs.
     */
    private static class InterpolationRecord implements Interpolatable<InterpolationRecord> {
        // The pose observed given the current sensor inputs and the previous pose.
        private final Pose2d poseMeters;

        // The current gyro angle.
        private final Rotation2d navXYaw;

        // The distance traveled by the left encoder.
        private final double globalXMeters;

        // The distance traveled by the right encoder.
        private final double globalYMeters;

        /**
         * Constructs an Interpolation Record with the specified parameters.
         *
         * @param poseMeters    The pose observed given the current sensor inputs and the previous pose.
         * @param navXYaw       The current navX yaw.
         * @param globalXMeters The total distance travelled by the NavX in meters.
         * @param globalYMeters The total distance travelled by the NavX in meters.
         */
        private InterpolationRecord(
            Pose2d poseMeters, Rotation2d navXYaw, double globalXMeters, double globalYMeters) {
            this.poseMeters = poseMeters;
            this.navXYaw = navXYaw;
            this.globalXMeters = globalXMeters;
            this.globalYMeters = globalYMeters;
        }

        /**
         * Return the interpolated record. This object is assumed to be the starting position, or lower
         * bound.
         *
         * @param endValue The upper bound, or end.
         * @param t        How far between the lower and upper bound we are. This should be bounded in [0, 1].
         * @return The interpolated value.
         */
        @Override
        public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                // Find the new global X traveled
                var global_x_lerp = MathUtil.interpolate(this.globalXMeters, endValue.globalXMeters, t);

                // Find the new global Y traveled
                var global_y_lerp = MathUtil.interpolate(this.globalYMeters, endValue.globalYMeters, t);

                // Find the new gyro angle.
                var gyro_lerp = navXYaw.interpolate(endValue.navXYaw, t);

                return new InterpolationRecord(new Pose2d(global_x_lerp, global_y_lerp, gyro_lerp), gyro_lerp, global_x_lerp, global_y_lerp);
            }
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof InterpolationRecord)) {
                return false;
            }
            InterpolationRecord record = (InterpolationRecord) obj;
            return Objects.equals(navXYaw, record.navXYaw)
                && Double.compare(globalXMeters, record.globalXMeters) == 0
                && Double.compare(globalYMeters, record.globalYMeters) == 0
                && Objects.equals(poseMeters, record.poseMeters);
        }

        @Override
        public int hashCode() {
            return Objects.hash(navXYaw, globalXMeters, globalYMeters, poseMeters);
        }
    }
}