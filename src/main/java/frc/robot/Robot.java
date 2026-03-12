package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TagDistanceHoldCommand;

public class Robot extends TimedRobot {

    private RobotContainer         m_robotContainer;
    private Command                m_autonomousCommand;
    private TagDistanceHoldCommand m_distHoldCommand;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.resetGyro();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.resetPivotToHome();
    }

    // ── Test 모드: AprilTag ID 10/26 거리 1m 유지 ────────────────
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        m_distHoldCommand = m_robotContainer.createDistHoldCommand();
        m_distHoldCommand.schedule();
    }

    @Override
    public void testPeriodic() {
        // CommandScheduler가 robotPeriodic()에서 이미 실행하므로 별도 코드 불필
    }

    @Override
    public void testExit() {
        if (m_distHoldCommand != null) {
            m_distHoldCommand.cancel();
        }
    }

    @Override public void disabledInit()       {}
    @Override public void disabledPeriodic()   {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic()     {}
}
