// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

/**
 * Fuses latency-compensated AprilTag vision measurements with swerve-drive odometry using a
 * continuous-time Kalman filter.
 *
 * <h3>Key differences from the WPILib standard {@code SwerveDrivePoseEstimator}:</h3>
 * <ul>
 *   <li><b>Multi-camera safe:</b> Vision updates from cameras running at different latencies are
 *       stored as independent, self-contained corrections. When a delayed frame from a slower
 *       camera arrives, future corrections from faster cameras are <em>not</em> discarded.
 *       Each {@link VisionUpdate} stores its own odometry snapshot so it remains valid
 *       regardless of when other cameras post their corrections.</li>
 *   <li><b>Adaptive outlier rejection:</b> Measurements farther from the current estimate than
 *       an acceptance radius are rejected. The radius starts at {@value #kBaseAcceptanceRadius}
 *       m and grows at {@value #kAcceptanceGrowthRate} m/s without a vision fix, so large but
 *       legitimate corrections after extended occlusion are still accepted.</li>
 *   <li><b>Per-update correction clamping:</b> The Kalman-scaled twist applied per update is
 *       clamped in both translation and rotation, preventing single-loop jumps even when a
 *       valid measurement is far from the current estimate. The estimate converges gradually
 *       over successive loops.</li>
 *   <li><b>Thread-safe:</b> All public methods are {@code synchronized} on the estimator
 *       instance, allowing the three camera threads and the swerve-drive periodic to call
 *       {@link #addVisionMeasurement} and {@link #updateWithTime} concurrently without
 *       data races on the vision-update map or Kalman matrices.</li>
 * </ul>
 */
public class NERDPoseEstimator {

    // ── Kalman filter state ────────────────────────────────────────────────────
    private final SwerveDriveOdometry m_odometry;
    /** Diagonal of the process-noise covariance Q (squared std devs). */
    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    /** Steady-state Kalman gain K (3×3 diagonal). */
    private final Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());

    // ── Buffer settings ────────────────────────────────────────────────────────
    /** How many seconds of odometry history to keep for latency compensation. */
    private static final double kBufferDuration = 1.5;

    // ── Outlier rejection ──────────────────────────────────────────────────────
    /**
     * Base acceptance radius (metres) when vision has been seen very recently.
     * Measurements further than this are rejected as outliers.
     */
    private static final double kBaseAcceptanceRadius = 0.7;
    /**
     * Rate (m/s) at which the acceptance radius grows when no vision update has
     * been received. Allows larger corrections after extended tag occlusion.
     */
    private static final double kAcceptanceGrowthRate = 0.3;
    /** Absolute cap on the acceptance radius regardless of occlusion duration. */
    private static final double kMaxAcceptanceRadius = 4.0;

    // ── Per-update correction clamping ─────────────────────────────────────────
    /**
     * Maximum translation correction (metres) applied in a single {@link
     * #addVisionMeasurement} call. Prevents visible single-frame jumps when
     * a large but valid correction arrives.
     */
    private static final double kMaxCorrectionMeters = 0.35;
    /**
     * Maximum rotation correction (radians) applied in a single {@link
     * #addVisionMeasurement} call.
     */
    private static final double kMaxCorrectionRadians = Math.PI / 6.0; // 30°

    // ── Internal state ─────────────────────────────────────────────────────────
    /**
     * Rolling 1.5-second buffer of odometry-only poses indexed by FPGA timestamp.
     * Used to look up the odometry pose at the time a camera frame was captured.
     */
    private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);

    /**
     * Map from FPGA timestamp → vision correction record.
     *
     * <p>Unlike the WPILib standard, this map is <em>not</em> pruned at the insertion
     * point when a new measurement arrives. Future corrections from faster cameras
     * coexist with delayed corrections from slower cameras.
     *
     * <p>There is always at most one entry per timestamp; if two cameras happen to
     * produce frames at the exact same FPGA timestamp the later one wins.
     */
    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    /** Best fused pose estimate, updated every {@link #updateWithTime} call. */
    private Pose2d m_poseEstimate;

    /**
     * FPGA timestamp of the most recently <em>accepted</em> vision measurement.
     * Used to compute the time-without-vision for the adaptive acceptance radius.
     * Initialised to {@link Double#NEGATIVE_INFINITY} so the radius starts at max.
     */
    private double m_lastVisionTimestamp = Double.NEGATIVE_INFINITY;

    // ══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ══════════════════════════════════════════════════════════════════════════

    @SuppressWarnings("PMD.UnusedFormalParameter")
    private NERDPoseEstimator(
            SwerveDriveKinematics kinematics,
            SwerveDriveOdometry odometry,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        m_odometry = odometry;
        m_poseEstimate = m_odometry.getPoseMeters();

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * Constructs a NERDPoseEstimator with default standard deviations.
     *
     * <p>State std devs: 0.1 m, 0.1 m, 0.1 rad.
     * Vision std devs: 0.9 m, 0.9 m, 0.9 rad.
     *
     * @param kinematics      A correctly-configured kinematics object.
     * @param gyroAngle       The current gyro angle.
     * @param modulePositions The current swerve module positions.
     * @param initialPoseMeters The starting pose estimate.
     */
    public NERDPoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        this(
                kinematics,
                gyroAngle,
                modulePositions,
                initialPoseMeters,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9));
    }

    /**
     * Constructs a NERDPoseEstimator with explicit standard deviations.
     *
     * @param kinematics               A correctly-configured kinematics object.
     * @param gyroAngle                The current gyro angle.
     * @param modulePositions          The current swerve module positions.
     * @param initialPoseMeters        The starting pose estimate.
     * @param stateStdDevs             Std devs of the odometry estimate [x, y, θ]. Increase to
     *                                 trust odometry less.
     * @param visionMeasurementStdDevs Std devs of the vision measurement [x, y, θ]. Increase to
     *                                 trust vision less.
     */
    public NERDPoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        this(
                kinematics,
                new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters),
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Configuration
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Updates the trust placed in global (vision) measurements.
     *
     * <p>Uses the closed-form steady-state Kalman gain for a continuous filter
     * with A = 0 and C = I: {@code K = Q / (Q + sqrt(Q·R))}.
     *
     * @param visionMeasurementStdDevs Std devs [x, y, θ] in metres and radians.
     */
    public final synchronized void setVisionMeasurementStdDevs(
            Matrix<N3, N1> visionMeasurementStdDevs) {
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                        row,
                        row,
                        m_q.get(row, 0)
                                / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
            }
        }
    }

    /**
     * Updates the trust placed in the odometry (state) estimate.
     *
     * @param odometryMeasurementStdDevs Std devs [x, y, θ] in metres and radians.
     */
    public final synchronized void setOdometryMeasurementStdDevs(
            Matrix<N3, N1> odometryMeasurementStdDevs) {
        for (int i = 0; i < 3; ++i) {
            m_q.set(
                    i,
                    0,
                    odometryMeasurementStdDevs.get(i, 0)
                            * odometryMeasurementStdDevs.get(i, 0));
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Reset
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Fully resets the pose estimator to a known pose, clearing all history.
     *
     * @param gyroAngle      The current gyro angle.
     * @param wheelPositions Current swerve module positions.
     * @param poseMeters     The known field-relative pose to reset to.
     */
    public synchronized void resetPosition(
            Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions,
            Pose2d poseMeters) {
        m_odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
        m_lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    }

    /**
     * Resets the pose, clearing all odometry and vision history.
     *
     * @param pose The new pose.
     */
    public synchronized void resetPose(Pose2d pose) {
        m_odometry.resetPose(pose);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
        m_lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    }

    /**
     * Resets only the translation component of the pose.
     *
     * @param translation The new translation.
     */
    public synchronized void resetTranslation(Translation2d translation) {
        m_odometry.resetTranslation(translation);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
        m_lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    }

    /**
     * Resets only the rotation component of the pose.
     *
     * @param rotation The new rotation.
     */
    public synchronized void resetRotation(Rotation2d rotation) {
        m_odometry.resetRotation(rotation);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
        m_lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    }

    /**
     * Resets only the underlying odometry pose (does not clear the vision buffer).
     * Use when drift correction via odometry reset is needed without losing vision history.
     *
     * @param pose Pose to reset odometry to.
     */
    public synchronized void resetOdometry(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

    /**
     * Resets odometry with full gyro + wheel-position context.
     *
     * @param gyroAngle      The current gyro angle.
     * @param wheelPositions Current swerve module positions.
     * @param poseMeters     Target odometry pose.
     */
    public synchronized void resetOdometryPosition(
            Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions,
            Pose2d poseMeters) {
        m_odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Getters
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Returns the best fused pose estimate (odometry + vision corrections).
     *
     * @return Estimated robot pose in metres.
     */
    public synchronized Pose2d getEstimatedPosition() {
        return m_poseEstimate;
    }

    /**
     * Returns the raw odometry pose (no vision correction applied).
     *
     * @return Odometry-only robot pose.
     */
    public synchronized Pose2d getOdometry() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Samples the fused pose estimate at a past FPGA timestamp, if the timestamp
     * lies within the 1.5-second odometry buffer.
     *
     * @param timestampSeconds FPGA timestamp in seconds.
     * @return Fused pose at that timestamp, or empty if the buffer has no data.
     */
    public synchronized Optional<Pose2d> sampleAt(double timestampSeconds) {
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        double oldestOdoTs = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        double newestOdoTs  = m_odometryPoseBuffer.getInternalBuffer().lastKey();
        timestampSeconds = MathUtil.clamp(timestampSeconds, oldestOdoTs, newestOdoTs);

        // No vision corrections yet, or the timestamp precedes all of them → pure odometry.
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return m_odometryPoseBuffer.getSample(timestampSeconds);
        }

        // Use the most recent vision correction at or before the queried timestamp.
        double floorTs = m_visionUpdates.floorKey(timestampSeconds);
        var visionUpdate = m_visionUpdates.get(floorTs);
        var odometryEstimate = m_odometryPoseBuffer.getSample(timestampSeconds);

        return odometryEstimate.map(visionUpdate::compensate);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Odometry update (call every loop)
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Updates the estimator with wheel-encoder and gyro data. Call every robot loop.
     *
     * @param gyroAngle      Current gyro angle.
     * @param wheelPositions Current swerve module positions.
     * @return The updated fused pose estimate.
     */
    public synchronized Pose2d update(
            Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        return updateWithTime(Timer.getFPGATimestamp(), gyroAngle, wheelPositions);
    }

    /**
     * Updates the estimator with wheel-encoder and gyro data at an explicit timestamp.
     * Call every robot loop.
     *
     * @param currentTimeSeconds FPGA time of this update call, in seconds.
     * @param gyroAngle          Current gyro angle.
     * @param wheelPositions     Current swerve module positions.
     * @return The updated fused pose estimate.
     */
    public synchronized Pose2d updateWithTime(
            double currentTimeSeconds,
            Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions) {
        var odometryEstimate = m_odometry.update(gyroAngle, wheelPositions);
        m_odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);

        if (m_visionUpdates.isEmpty()) {
            m_poseEstimate = odometryEstimate;
        } else {
            var latestVisionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate = latestVisionUpdate.compensate(odometryEstimate);
        }

        return m_poseEstimate;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Vision measurement fusion
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Fuses a vision pose measurement into the estimator.
     *
     * <h4>Multi-camera correctness</h4>
     * <p>Unlike the WPILib standard implementation, this method does <em>not</em>
     * clear vision updates timestamped after {@code timestampSeconds}. With three
     * cameras running at different framerates and latencies, a slow camera processing
     * an old frame must not invalidate corrections already posted by faster cameras.
     * Each {@link VisionUpdate} is self-contained — it stores the raw odometry pose
     * at the time it was created — so later updates do not need to be recomputed when
     * an earlier one is inserted.
     *
     * <h4>Outlier rejection</h4>
     * <p>Measurements whose translation is more than {@code acceptanceRadius} metres
     * from the current estimate are silently dropped. The radius begins at
     * {@value #kBaseAcceptanceRadius} m and grows at {@value #kAcceptanceGrowthRate}
     * m/s without a successful update, capped at {@value #kMaxAcceptanceRadius} m.
     * This prevents bad-ambiguity or wrong-tag frames from corrupting the estimate
     * while still allowing large corrections after extended occlusion.
     *
     * <h4>Correction clamping</h4>
     * <p>Even after the Kalman gain has scaled the correction, its translation
     * magnitude is clamped to {@value #kMaxCorrectionMeters} m and its rotation to
     * ±{@value #kMaxCorrectionRadians} rad per call. Larger errors converge smoothly
     * over successive loops rather than jumping in a single frame.
     *
     * @param visionRobotPoseMeters Field-relative robot pose from the vision pipeline.
     * @param timestampSeconds      FPGA timestamp of the camera frame, in seconds.
     */
    public synchronized void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds) {

        // ── Guard: frame too old for the latency-compensation buffer ───────────
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
                || m_odometryPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration
                        > timestampSeconds) {
            System.out.println("VISION RETURN: frame too old");

            return;
        }

        cleanUpVisionUpdates();

        // ── Odometry pose at the moment the camera captured this frame ─────────
        var odometrySample = m_odometryPoseBuffer.getSample(timestampSeconds);
        if (odometrySample.isEmpty()) {
            System.out.println("VISION RETURN: odometry sample empty");
            return;
        }

        // ── Best fused estimate at that same moment ────────────────────────────
        var visionSample = sampleAt(timestampSeconds);
        if (visionSample.isEmpty()) {
            System.out.println("VISION RETURN: vision sample empty");
            return;}

        // ── Adaptive outlier rejection ─────────────────────────────────────────
        // The acceptance radius grows with time since the last accepted update so
        // that legitimate corrections after extended tag occlusion are still applied.
        double now = Timer.getFPGATimestamp();
        double timeSinceVision = Math.max(0.0, now - m_lastVisionTimestamp);
        double acceptanceRadius = Math.min(
                kBaseAcceptanceRadius + kAcceptanceGrowthRate * timeSinceVision,
                kMaxAcceptanceRadius);

        double distanceFromEstimate = m_poseEstimate.getTranslation()
                .getDistance(visionRobotPoseMeters.getTranslation());
        if (distanceFromEstimate > acceptanceRadius) {
            // System.out.println("VISION RETURN: outlier, " + (distanceFromEstimate - acceptanceRadius));

            // return; // Outlier — too far from the current estimate
        }

        // ── Kalman-weighted correction twist ───────────────────────────────────
        // Compute the twist from the best fused estimate (at frame time) to the
        // raw vision measurement, then scale by the Kalman gain.
        var twist = visionSample.get().log(visionRobotPoseMeters);
        var kTimesTwist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

        double dx     = kTimesTwist.get(0, 0);
        double dy     = kTimesTwist.get(1, 0);
        double dtheta = kTimesTwist.get(2, 0);

        // ── Per-update correction clamping ─────────────────────────────────────
        // Cap translation magnitude so a large (but valid) jump is spread across
        // multiple loops, keeping the displayed pose smooth.
        double translationNorm = Math.hypot(dx, dy);
        // if (translationNorm > kMaxCorrectionMeters) {
        //     double scale = kMaxCorrectionMeters / translationNorm;
        //     dx *= scale;
        //     dy *= scale;
        // }
        // dtheta = MathUtil.clamp(dtheta, -kMaxCorrectionRadians, kMaxCorrectionRadians);

        var scaledTwist = new Twist2d(dx, dy, dtheta);

        // ── Store vision correction ────────────────────────────────────────────
        // visionPose   = fused estimate at frame time + Kalman-scaled correction
        // odometryPose = raw odometry at frame time (used as the delta reference)
        var visionUpdate = new VisionUpdate(
                visionSample.get().exp(scaledTwist), odometrySample.get());
        m_visionUpdates.put(timestampSeconds, visionUpdate);

        // ── DO NOT clear future vision updates ─────────────────────────────────
        // The standard WPILib approach clears m_visionUpdates.tailMap(ts, false)
        // here, which is correct for a single camera (future samples were computed
        // from the now-stale pose at ts). With three cameras at different latencies,
        // however, those "future" entries were posted by other cameras independently
        // and are still correct — each stores its own odometry reference. Clearing
        // them when a slow-camera delayed frame arrives causes the estimate to snap
        // back to the slow camera's view, producing the jumpiness this rewrite fixes.

        // ── Advance the running estimate ───────────────────────────────────────
        // Always use the most recent correction (largest key) to project forward
        // to the current odometry position.
        var latestUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
        m_poseEstimate = latestUpdate.compensate(m_odometry.getPoseMeters());

        m_lastVisionTimestamp = now;
    }

    /**
     * Fuses a vision measurement with explicit per-measurement standard deviations.
     *
     * <p>The provided std devs update the global Kalman gain and persist until the
     * next call to this method or {@link #setVisionMeasurementStdDevs}. Since every
     * camera call in this codebase supplies its own std devs (scaled by distance and
     * ambiguity), this is the expected usage path.
     *
     * @param visionRobotPoseMeters    Field-relative robot pose from the vision pipeline.
     * @param timestampSeconds         FPGA timestamp of the camera frame, in seconds.
     * @param visionMeasurementStdDevs Std devs [x, y, θ] for this measurement.
     */
    public synchronized void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // Internal helpers
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Removes vision updates that are older than the oldest odometry sample still
     * in the buffer. One entry immediately before the oldest odometry timestamp is
     * kept so that {@link #sampleAt} can interpolate correctly at the buffer edge.
     */
    private void cleanUpVisionUpdates() {
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            // System.out.println("VISION RETURN: odometry return empty");
            return;
        }

        double oldestOdoTs = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        if (m_visionUpdates.isEmpty() || oldestOdoTs < m_visionUpdates.firstKey()) {
            // System.out.println("VISION RETURN: odometry return empty");
            return;
        }

        
        double newestNeeded = m_visionUpdates.floorKey(oldestOdoTs);
        var updatesToPrune = m_visionUpdates.headMap(newestNeeded, false);
        updatesToPrune.clear();
        System.out.println(m_visionUpdates.size() + " updates left from " + (Timer.getFPGATimestamp()-newestNeeded) + "seconds ago");
    }

    // ══════════════════════════════════════════════════════════════════════════
    // VisionUpdate record
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Stores the Kalman-corrected pose alongside the raw odometry pose at the time a
     * vision measurement was applied. Used to project the correction forward to the
     * current odometry position via {@link #compensate}.
     *
     * <p>The projection formula is:
     * <pre>
     *   estimate(t_now) = visionPose + (currentOdometry − odometryPose)
     * </pre>
     * i.e., take the corrected pose at the camera frame time and add the odometry
     * delta that has accumulated since then.
     */
    private static final class VisionUpdate {
        /** Kalman-fused pose at the time the vision measurement was applied. */
        private final Pose2d visionPose;
        /** Raw odometry pose at the same timestamp (delta reference). */
        private final Pose2d odometryPose;

        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Projects this correction to a later odometry position.
         *
         * @param pose The current (or queried) odometry pose.
         * @return The vision-corrected pose at that odometry position.
         */
        public Pose2d compensate(Pose2d pose) {
            var delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
}
