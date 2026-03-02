package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    /**
     * PhotonVision 대시보드의 카메라 이름과 반드시 일치시켜야 한다.
     * 대시보드 접속 후 Cameras 탭에서 카메라 이름 확인 후 수정하세요.
     */
    private final PhotonCamera m_camera = new PhotonCamera("OV9281");

    /** AprilTag 타겟이 시야에 있는지 여부 */
    public boolean hasTarget() {
        return m_camera.getLatestResult().hasTargets();
    }

    /**
     * 카메라 중심 기준 좌우 각도 오차 (Yaw, degree)
     * 양수 = 타겟이 왼쪽, 음수 = 타겟이 오른쪽
     * 타겟이 없으면 0.0 반환
     */
    public double getTargetYaw() {
        var result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }
}
