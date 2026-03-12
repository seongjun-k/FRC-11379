package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Test 모드 전용 - AprilTag ID 10 또는 26이 보이면
 * 목표 거리(TARGET_DIST_M)를 유지하면서 Yaw 정렬도 동시 수행
 *
 * • 타겟 없으면: 정지
 * • 너무 멀면: 전진 (0.3)
 * • 너무 가까우면: 후진 (-0.3)
 * • Yaw 오차가 있으면: 회전 (0.3 최대)
 */
public class TagDistanceHoldCommand extends Command {

    private static final int    TAG_A          = 10;
    private static final int    TAG_B          = 26;
    private static final double TARGET_DIST_M  = 1.0;   // 목표 거리 (m)
    private static final double DIST_TOLERANCE = 0.08;  // ±0.08m 이내면 전후진 정지
    private static final double YAW_TOLERANCE  = 2.0;   // ±2° 이내면 회전 정지

    private static final double MAX_DRIVE  = 0.3;  // 최대 직진 출력
    private static final double MAX_ROTATE = 0.3;  // 최대 회전 출력

    private static final double DIST_P  = 0.4;  // 거리 P게인
    private static final double YAW_P   = 0.02; // Yaw P게인

    private final DriveSubsystem  m_drive;
    private final VisionSubsystem m_vision;

    public TagDistanceHoldCommand(DriveSubsystem drive, VisionSubsystem vision) {
        m_drive  = drive;
        m_vision = vision;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double dist = m_vision.getTargetDistanceById(TAG_A, TAG_B);
        double yaw  = m_vision.getTargetYawById(TAG_A, TAG_B);

        if (Double.isNaN(dist)) {
            // 타겟 없음 → 정지
            m_drive.arcadeDrive(0, 0);
            SmartDashboard.putString("DistHold/Status", "NO TARGET");
            return;
        }

        // 거리 오차: 양수 = 너무 멀다 = 전진
        double distError = dist - TARGET_DIST_M;
        double driveOut  = 0.0;
        if (Math.abs(distError) > DIST_TOLERANCE) {
            driveOut = distError * DIST_P;
            // 최대 출력 제한
            driveOut = Math.max(-MAX_DRIVE, Math.min(MAX_DRIVE, driveOut));
        }

        // Yaw 오차: 양수 = 왼쪽 = 시계방향 회전 필요
        double rotOut = 0.0;
        if (Math.abs(yaw) > YAW_TOLERANCE) {
            rotOut = -yaw * YAW_P;
            rotOut = Math.max(-MAX_ROTATE, Math.min(MAX_ROTATE, rotOut));
        }

        m_drive.arcadeDrive(driveOut, rotOut);

        SmartDashboard.putNumber("DistHold/Distance (m)",  dist);
        SmartDashboard.putNumber("DistHold/DistError (m)", distError);
        SmartDashboard.putNumber("DistHold/DriveOut",      driveOut);
        SmartDashboard.putNumber("DistHold/RotOut",        rotOut);
        SmartDashboard.putString("DistHold/Status",        "TRACKING");
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
        SmartDashboard.putString("DistHold/Status", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false; // testInit에서 cancel()로 종료
    }
}
