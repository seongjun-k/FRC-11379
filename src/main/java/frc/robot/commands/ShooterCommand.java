package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem m_shooter;
    private final DoubleSupplier   m_rtSupplier;
    private final DoubleSupplier   m_ltSupplier;
    private final BooleanSupplier  m_povUpSupplier;
    private final BooleanSupplier  m_povDownSupplier;

    /** D-pad으로 설정한 슈터 목표 속도 (0.0 ~ 1.0, 5% 단위) */
    private double  m_speedTarget = 0.8;
    private boolean m_prevPovUp   = false;
    private boolean m_prevPovDown = false;

    /**
     * @param shooter       슈터 서브시스템
     * @param rt            오른쪽 트리거 (정방향 발사)
     * @param lt            왼쪽 트리거  (역방향)
     * @param povUp         D-pad 위 버튼 (속도 +5%)
     * @param povDown       D-pad 아래 버튼 (속도 -5%)
     */
    public ShooterCommand(ShooterSubsystem shooter,
                          DoubleSupplier rt,
                          DoubleSupplier lt,
                          BooleanSupplier povUp,
                          BooleanSupplier povDown) {
        m_shooter         = shooter;
        m_rtSupplier      = rt;
        m_ltSupplier      = lt;
        m_povUpSupplier   = povUp;
        m_povDownSupplier = povDown;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // ── D-pad 상승 엣지 감지 (버튼을 누른 순간만 1회 반응) ──────────
        boolean povUp   = m_povUpSupplier.getAsBoolean();
        boolean povDown = m_povDownSupplier.getAsBoolean();

        if (povUp && !m_prevPovUp) {
            m_speedTarget = Math.min(1.0, Math.round((m_speedTarget + 0.05) * 100.0) / 100.0);
        }
        if (povDown && !m_prevPovDown) {
            m_speedTarget = Math.max(0.0, Math.round((m_speedTarget - 0.05) * 100.0) / 100.0);
        }

        m_prevPovUp   = povUp;
        m_prevPovDown = povDown;

        // ── 출력 계산: RT/LT = on/off 트리거, 속도 = m_speedTarget ──────
        double rt = m_rtSupplier.getAsDouble();
        double lt = m_ltSupplier.getAsDouble();

        double output;
        if (rt > 0.05) {
            output =  m_speedTarget;
        } else if (lt > 0.05) {
            output = -m_speedTarget;
        } else {
            output = 0.0;
        }

        SmartDashboard.putNumber("Shooter/SpeedTarget %", m_speedTarget * 100.0);
        m_shooter.setOutput(output);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
