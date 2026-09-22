package org.team100.lib.subsystems.swerve;

import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.AprilTagCornerRobotLocalizer;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Pull together some of the drivetrain's dependencies so they don't pollute
 * Robot.java.
 */
public class SwerveDriveFactory {

    public static SwerveDriveSubsystem get(
            LoggerFactory driveLog,
            LoggerFactory fieldLogger,
            SwerveKinodynamics swerveKinodynamics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Gyro gyro,
            SwerveModuleCollection modules) {
        SwerveHistory history = new SwerveHistory(
                driveLog,
                swerveKinodynamics,
                0.2,
                gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                modules.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());
        OdometryUpdater odometryUpdater = OdometryUpdater.normal(
                driveLog,
                swerveKinodynamics,
                gyro,
                history,
                modules::positions);
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
        SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                swerveKinodynamics,
                modules);
        return new SwerveDriveSubsystem(
                driveLog,
                odometryUpdater,
                estimate,
                swerveLocal);
    }
}
