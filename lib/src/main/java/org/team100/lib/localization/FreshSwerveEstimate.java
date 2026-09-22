package org.team100.lib.localization;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Updates the vision and odometry before sampling the history.
 * 
 * Proxy the history after making sure it has received any updates that may
 * mutate it. Some clients want "fresh" estimates, and should use this class;
 * other clients only need old historical estimates, and should use the history.
 */
public class FreshSwerveEstimate {
    private static final boolean DEBUG = false;

    /** SwerveHistory delegate. */
    private final StateSampler m_history;
    private final AprilTagCornerRobotLocalizer m_localizer;
    private final OdometryUpdater m_odometryUpdate;
    /** Side effect mutates history. */
    private final SideEffect m_localizerCache;
    /** Side effect mutates history. */
    private final SideEffect m_odometryCache;

    public FreshSwerveEstimate(
            AprilTagCornerRobotLocalizer localizer,
            OdometryUpdater odometryUpdate,
            StateSampler history) {
        m_localizer = localizer;
        m_odometryUpdate = odometryUpdate;
        m_history = history;
        m_localizerCache = Cache.ofSideEffect(localizer::update);
        m_odometryCache = Cache.ofSideEffect(odometryUpdate::update);
    }

    public static FreshSwerveEstimate get(
            LoggerFactory driveLog,
            LoggerFactory fieldLogger,
            SwerveKinodynamics swerveKinodynamics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Gyro gyro,
            SwerveLocal swerveLocal) {
        SwerveHistory history = new SwerveHistory(
                driveLog,
                swerveKinodynamics,
                0.2,
                gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                swerveLocal.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());
        OdometryUpdater odometryUpdater = OdometryUpdater.normal(
                driveLog,
                swerveKinodynamics,
                gyro,
                history,
                swerveLocal::positions);
        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                driveLog, history, odometryUpdater);
        AprilTagCornerRobotLocalizer localizer = new AprilTagCornerRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                visionUpdater,
                DriverStation::getAlliance);
        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer,
                odometryUpdater,
                history);
        return estimate;
    }

    /** Noiseless for testing */
    public static FreshSwerveEstimate test(
            LoggerFactory driveLog,
            LoggerFactory fieldLogger,
            SwerveKinodynamics swerveKinodynamics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Gyro gyro,
            SwerveLocal swerveLocal) {
        SwerveHistory history = new SwerveHistory(
                driveLog,
                swerveKinodynamics,
                0.2,
                gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                swerveLocal.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());
        OdometryUpdater odometryUpdater = OdometryUpdater.noiseless(
                driveLog,
                swerveKinodynamics,
                gyro,
                history,
                swerveLocal::positions);
        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                driveLog, history, odometryUpdater);
        AprilTagCornerRobotLocalizer localizer = new AprilTagCornerRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                visionUpdater,
                DriverStation::getAlliance);
        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer,
                odometryUpdater,
                history);
        return estimate;
    }

    /**
     * Provide the best estimate for SwerveModel at the given timestamp, first
     * making sure any pending updates from vision or odometry have been applied.
     * 
     * The estimate is used for many things downstream; noise there is bad.
     * The estimator itself should have enough controls to make the estimate
     * arbitrarily smooth.
     */
    public StateSE2 apply(double timestampS) {
        // run our dependencies if they haven't already
        m_localizerCache.run();
        m_odometryCache.run();
        // query the history
        StateSE2 state = m_history.apply(timestampS);
        if (DEBUG) {
            System.out.printf("FreshSwerveEstimate.update() estimated pose: %s\n", state);
        }
        return state;
    }

    /**
     * Empty the pose history, reset the servos, add the given pose, and flush the
     * cache.
     */
    public void reset(Pose2d robotPose, IsotropicNoiseSE2 noise) {
        m_odometryUpdate.reset(robotPose, noise);
        reset();
    }

    /**
     * Invalidate the caches, so the next apply() will poll the delegates.
     */
    public void reset() {
        m_localizerCache.reset();
        m_odometryCache.reset();
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_localizer.setHeedRadiusM(heedRadiusM);
    }

}
