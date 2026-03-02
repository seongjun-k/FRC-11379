package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 드라이버가 X 버튼을 누르고 있는 동안
 * 제자리에서 AprilTag 방향으로 자동 정렬하는 커맨드.
 *
 * 튜닝 가이드:
 *   P_GAIN:       진동하면 낮춰라(0.01), 느리면 올려라(0.05)
 *   TOLERANCE_DEG: 원하는 정지 정밀도에 따라 조절 (1.0 ~ 3.0)
 */
public class AutoAlignCommand extends Command {

    private final DriveSubsystem  m_drive;
    private final VisionSubsystem m_vision;

    private static final double P_GAIN       = 0.03;
    private static final double TOLERANCE_DEG = 1.5;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        m_drive  = drive;
        m_vision = vision;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        if (m_vision.hasTarget()) {
            // Yaw 값이 양수(왼쪽) 이면 시계 방향으로 회전 필요 → 음수 회전 출력
            double rotation = -m_vision.getTargetYaw() * P_GAIN;
            m_drive.arcadeDrive(0, rotation);
        } else {
            // 타겟이 없으면 정지
            m_drive.arcadeDrive(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        // 타겟이 보이고 각도 오차가 허용 범위 이내면 자동 종료
        return m_vision.hasTarget()
            && Math.abs(m_vision.getTargetYaw()) < TOLERANCE_DEG;
    }
}
