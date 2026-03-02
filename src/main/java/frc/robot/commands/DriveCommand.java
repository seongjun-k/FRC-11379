package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_zRotation;

    // ── 직진 보정용 ────────────────────────────────
    private double m_targetAngle = 0.0;

    /**
     * 직진 보정용 P 게인: 자이로 각도 오차(deg) 1당 회전 출력 비율
     * 진동이 생기면 낮춰라 (0.01), 너무 느리면 올려라 (0.05)
     */
    private static final double STRAIGHT_KP = 0.02;

    /** 조이스틱 회전 입력이 이 값 미만이면 직진 보정 모드 활성화 */
    private static final double TURN_DEADBAND    = 0.05;

    /** 이 속도 이상일 때만 직진 보정 활성화 */
    private static final double FORWARD_DEADBAND = 0.1;

    public DriveCommand(
        DriveSubsystem drive,
        DoubleSupplier xSpeed,
        DoubleSupplier zRotation
    ) {
        m_drive     = drive;
        m_xSpeed    = xSpeed;
        m_zRotation = zRotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double forward = m_xSpeed.getAsDouble();
        double turn    = m_zRotation.getAsDouble();

        if (Math.abs(turn) < TURN_DEADBAND && Math.abs(forward) > FORWARD_DEADBAND) {
            // 직진 모드: 자이로가 틀어진 만큼 반대 방향으로 자동 보정
            double headingError = m_targetAngle - m_drive.getGyroAngle();
            turn = headingError * STRAIGHT_KP;
        } else {
            // 회전 조작 중: 현재 각도를 타겟 각도로 지속 갱신
            m_targetAngle = m_drive.getGyroAngle();
        }

        m_drive.arcadeDrive(forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false; // 텔레옵 중 계속 실행
    }
}
