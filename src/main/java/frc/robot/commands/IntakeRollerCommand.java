package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * 인테이크 롤러 커맨드
 * 버튼을 누르는 동안 롤러를 구동하고, 떼면 정지합니다.
 *
 * ★ 방향 반전이 필요하면:
 *   Constants.IntakeConstants.ROLLER_INVERTED 를 true 로 변경하세요.
 */
public class IntakeRollerCommand extends Command {

    private final IntakeRollerSubsystem m_intake;

    public IntakeRollerCommand(IntakeRollerSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // ROLLER_INVERTED == true 이면 음수 방향으로 구동
        double direction = IntakeConstants.ROLLER_INVERTED ? -1.0 : 1.0;
        m_intake.setSpeed(direction * IntakeConstants.ROLLER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // whileTrue 바인딩과 함께 사용
    }
}
