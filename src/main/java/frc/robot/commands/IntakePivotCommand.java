package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

/**
 * 인테이크 피벗 커맨드
 * direction: +1.0 = 위(정회전), -1.0 = 아래(역회전)
 * 버튼을 누르는 동안 동작하고 떼면 정지합니다.
 */
public class IntakePivotCommand extends Command {

    public enum Direction {
        UP, DOWN
    }

    private final IntakePivotSubsystem m_pivot;
    private final Direction m_direction;

    public IntakePivotCommand(IntakePivotSubsystem pivot, Direction direction) {
        m_pivot     = pivot;
        m_direction = direction;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        if (m_direction == Direction.UP) {
            m_pivot.pivotUp();
        } else {
            m_pivot.pivotDown();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
