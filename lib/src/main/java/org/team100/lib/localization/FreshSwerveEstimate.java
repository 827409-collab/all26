package org.team100.lib.localization;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.state.StateSE2;

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
    /** Side effect mutates history. */
    private final SideEffect m_vision;
    /** Side effect mutates history. */
    private final SideEffect m_odometry;

    /**
     * @param visionUpdate   AprilTagRobotLocalizer::update
     * @param odometryUpdate OdometryUpdater::update
     * @param history        SwerveHistory
     */
    public FreshSwerveEstimate(
            AprilTagCornerRobotLocalizer localizer,
            OdometryUpdater odometryUpdate,
            StateSampler history) {
        m_localizer = localizer;
        m_history = history;
        m_vision = Cache.ofSideEffect(localizer::update);
        m_odometry = Cache.ofSideEffect(odometryUpdate::update);
    }

    /**
     * Provide the best estimate for SwerveModel at the given timestamp, first
     * making sure any pending updates from vision or odometry have been applied.
     */
    public StateSE2 apply(double timestampS) {
        // run our dependencies if they haven't already
        m_vision.run();
        m_odometry.run();
        // query the history
        StateSE2 state = m_history.apply(timestampS);
        if (DEBUG) {
            System.out.printf("FreshSwerveEstimate.update() estimated pose: %s\n", state);
        }
        return state;
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_localizer.setHeedRadiusM(heedRadiusM);
    }

}
