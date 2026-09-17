package org.team100.frc2026.auton;

import org.junit.jupiter.api.Test;
import org.team100.frc2026.robot.Machinery;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.se2.TrajectorySE2;

public class DoubleCircleAutonTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(log);
    private static final SwerveKinodynamics dynamics = SwerveKinodynamicsFactory.get();
    private static final ControllerSE2 controller = ControllerFactorySE2.byIdentity(log);
    private static final Machinery machinery = new Machinery(currentLog);

    @Test
    void test0() {
        DoubleCircleAuton a = new DoubleCircleAuton(
                log, dynamics, controller, machinery.m_solver, machinery);
        TrajectorySE2 t = a.t1(a.start());
        t.getLastPoint();
    }

}
