package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer m_robotContainer;
    private Command        m_autonomousCommand;

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
        // 오토 커맨드 강제 취소
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // 오토에서 피벗이 내려간 채로 끝났을 경우를 대비해 HOME 위치로 목표 리셋
        // (실제 모터가 올라가진 않고, PID 목표값만 HOME으로 바꿔서 텔레옵 시작 시 올라감)
        m_robotContainer.resetPivotToHome();
    }

    @Override public void disabledInit()       {}
    @Override public void disabledPeriodic()   {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic()     {}
    @Override public void testInit()           { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testPeriodic()       {}
}
