package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

/**
 * 인테이크 피벗 커맨드 (Position PID 기반)
 *
 * 동작:
 *   DOWN  버튼 누름 → deployIntake()  : INTAKE_POS로 PID 이동
 *   UP    버튼 누름 → retractIntake() : HOME_POS로 PID 이동
 *   버튼 뗌 (end) → holdCurrentPosition() : 그 자리에서 kG로 버팀
 *
 *   BUMP_DOWN / BUMP_UP: 까닥까닥 미세 조정용
 */
public class IntakePivotCommand extends Command {

    public enum Direction {
        DOWN,
        UP,
        BUMP_DOWN,
        BUMP_UP
    }

    private final IntakePivotSubsystem m_pivot;
    private final Direction            m_direction;

    public IntakePivotCommand(IntakePivotSubsystem pivot, Direction direction) {
        m_pivot     = pivot;
        m_direction = direction;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        // BUMP는 initialize()에서 한 번만 실행 (한 번 까닥)
        if (m_direction == Direction.BUMP_DOWN) m_pivot.bumpDown();
        if (m_direction == Direction.BUMP_UP)   m_pivot.bumpUp();
    }

    @Override
    public void execute() {
        switch (m_direction) {
            case DOWN:      m_pivot.deployIntake();  break;
            case UP:        m_pivot.retractIntake(); break;
            // BUMP는 initialize()에서 처리했으므로 execute()에서는 아무것도 안 함
            default: break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // ★ 버튼 떼면 stop()이 아닌 holdCurrentPosition()
        // → 중력 보상(kG)으로 제자리 유지, 탕! 현상 없음
        m_pivot.holdCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        // BUMP는 목표 설정 후 즉시 종료 (end()에서 자동으로 holdCurrentPosition() 호출)
        if (m_direction == Direction.BUMP_DOWN || m_direction == Direction.BUMP_UP) {
            return true;
        }
        return false;
    }
}
