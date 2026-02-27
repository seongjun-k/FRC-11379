package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * 컨베이어 커맨드
 * 버튼을 누르는 동안 컨베이어를 구동하고, 떼면 정지합니다.
 *
 * ★ 방향 반전이 필요하면:
 *   Constants.ConveyorConstants.CONVEYOR_INVERTED 를 true 로 변경하세요.
 */
public class ConveyorCommand extends Command {

    private final ConveyorSubsystem m_conveyor;

    public ConveyorCommand(ConveyorSubsystem conveyor) {
        m_conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        // CONVEYOR_INVERTED == true 이면 음수 방향으로 구동
        double direction = ConveyorConstants.CONVEYOR_INVERTED ? -1.0 : 1.0;
        m_conveyor.setSpeed(direction * ConveyorConstants.CONVEYOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
