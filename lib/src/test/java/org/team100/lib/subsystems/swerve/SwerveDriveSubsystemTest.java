package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.localization.AprilTagCornerRobotLocalizer;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

class SwerveDriveSubsystemTest implements Timeless {

    private static final double DELTA = 0.01;

    @Test
    void testWithSetpointGenerator() throws IOException {

        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        SwerveHistory history = new SwerveHistory(
                logger,
                swerveKinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater odometryUpdater = new OdometryUpdater(
                logger, swerveKinodynamics, gyro, history,
                collection::positions, UnaryOperator.identity(), true);
        odometryUpdater.reset(Pose2d.kZero, IsotropicNoiseSE2.high(), 0);

        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                logger, history, odometryUpdater);

        final AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagCornerRobotLocalizer localizer = new AprilTagCornerRobotLocalizer(
                logger, fieldLogger, layout, history, visionUpdater, DriverStation::getAlliance);
        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer, odometryUpdater::update, history);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                odometryUpdater,
                estimate,
                swerveLocal);

        Experiments.INSTANCE.override(Experiment.UseSwerveLimiter, true);

        drive.resetPose(new Pose2d(), IsotropicNoiseSE2.high());

        stepTime();
        drive.periodic();
        verify(drive, 0, 0, 0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), ChassisAcceleration.ZERO);

        // actuation is reflected in measurement after time passes
        assertEquals(0, collection.states().frontLeft().speed());
        stepTime();
        assertEquals(1, collection.states().frontLeft().speed());

        drive.periodic();
        assertEquals(0.02, collection.positions().frontLeft().distanceMeters(), 1e-6);

        assertEquals(1, collection.states().frontLeft().speed());

        // the acceleration limit is applied here
        verify(drive, 0.02, 1, 1.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        drive.periodic();

        verify(drive, 0.039, 1, 1.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        drive.periodic();

        verify(drive, 0.06, 1, 0.06);

        drive.close();
    }

    @Test
    void testWithoutSetpointGenerator() throws IOException {

        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        SwerveHistory history = new SwerveHistory(
                logger,
                swerveKinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater odometryUpdater = new OdometryUpdater(
                logger, swerveKinodynamics, gyro, history,
                collection::positions, UnaryOperator.identity(), true);
        odometryUpdater.reset(Pose2d.kZero, IsotropicNoiseSE2.high(), 0);

        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                logger, history, odometryUpdater);

        final AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagCornerRobotLocalizer localizer = new AprilTagCornerRobotLocalizer(
                logger, fieldLogger, layout, history, visionUpdater, DriverStation::getAlliance);
        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer, odometryUpdater::update, history);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                odometryUpdater,
                estimate,
                swerveLocal);

        Experiments.INSTANCE.override(Experiment.UseSwerveLimiter, false);
        stepTime();

        drive.resetPose(new Pose2d(), IsotropicNoiseSE2.high());

        stepTime();
        drive.periodic();

        verify(drive, 0, 0, 0);

        // go 1 m/s in +x
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        drive.periodic();

        // at 1 m/s for 0.02 s, so we go 0.02 m
        assertEquals(0.02, collection.positions().frontLeft().distanceMeters(), 1e-6);

        // it took 0.02 s to go from 0 m/s to 1 m/s, so we accelerated 50 m/s/s.
        verify(drive, 0.02, 1.00, 50.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        drive.periodic();

        // we went a little further, no longer accelerating.
        verify(drive, 0.04, 1.00, 0.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        drive.periodic();

        // a little further, but no longer accelerating
        verify(drive, 0.06, 1.00, 0.0);

        drive.close();
    }

    private void verify(SwerveDriveSubsystem drive, double x, double v, double a) {
        assertEquals(x, drive.getPose().getX(), DELTA);
        assertEquals(v, drive.getVelocity().x(), DELTA);
        // assertEquals(a, drive.getState().acceleration().x(), DELTA);
        assertEquals(x, drive.getState().x().x(), DELTA);
        assertEquals(v, drive.getState().x().v(), DELTA);
        // assertEquals(a, drive.getState().x().a(), DELTA);
    }
}
