package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * 컨베이어 커맨드
 * 버튼을 누르는 동안 컨베이어를 구동하고, 떼면 정지합니다.
 * 
 * 패턴: [돌기 0.1초] → [멈춤 0.1초] → [돌기 0.1초] → ... 반복
 * 타이밍은 Constants에서 조정 가능합니다:
 *   - CONVEYOR_RUN_TIME: 구동 시간 (초)
 *   - CONVEYOR_STOP_TIME: 정지 시간 (초)
 *
 * ★ 방향 반전이 필요하면:
 *   Constants.ConveyorConstants.CONVEYOR_INVERTED 를 true 로 변경하세요.
 */
public class ConveyorCommand extends Command {

    private final ConveyorSubsystem m_conveyor;
    private final Timer m_timer;

    public ConveyorCommand(ConveyorSubsystem conveyor) {
        m_conveyor = conveyor;
        m_timer = new Timer();
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        double cycleTime = m_timer.get() % (ConveyorConstants.CONVEYOR_RUN_TIME + ConveyorConstants.CONVEYOR_STOP_TIME);

        if (cycleTime < ConveyorConstants.CONVEYOR_RUN_TIME) {
            // 구동 상태: 모터 회전
            double direction = ConveyorConstants.CONVEYOR_INVERTED ? -1.0 : 1.0;
            m_conveyor.setSpeed(direction * ConveyorConstants.CONVEYOR_SPEED);
        } else {
            // 정지 상태: 모터 멈춤
            m_conveyor.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.stop();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
