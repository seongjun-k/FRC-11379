package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera m_camera = new PhotonCamera("OV9281");

    private static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.5),
        new Rotation3d(0, 0, 0)
    );

    private final PhotonPoseEstimator m_photonEstimator;

    public VisionSubsystem() {
        var layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        m_photonEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_OFFSET
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return m_photonEstimator.update(m_camera.getLatestResult());
    }

    public boolean hasTarget() {
        return m_camera.getLatestResult().hasTargets();
    }

    public double getTargetYaw() {
        var result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    /**
     * 지정 ID 중 하나가 보이면 Yaw 반환, 없으면 NaN
     */
    public double getTargetYawById(int... ids) {
        var result = m_camera.getLatestResult();
        if (!result.hasTargets()) return Double.NaN;
        for (PhotonTrackedTarget t : result.getTargets()) {
            for (int id : ids) {
                if (t.getFiducialId() == id) return t.getYaw();
            }
        }
        return Double.NaN;
    }

    /**
     * 지정 ID 중 하나가 보이면 카메라 기준 수평 거리(m) 반환, 없으면 NaN
     * PhotonVision getBestCameraToTarget() 의 Translation X = 정면 거리
     */
    public double getTargetDistanceById(int... ids) {
        var result = m_camera.getLatestResult();
        if (!result.hasTargets()) return Double.NaN;
        for (PhotonTrackedTarget t : result.getTargets()) {
            for (int id : ids) {
                if (t.getFiducialId() == id) {
                    Transform3d camToTarget = t.getBestCameraToTarget();
                    // X: 정면 거리, Y: 좌우 오프셋
                    // 수평 직선 거리 = sqrt(X^2 + Y^2)
                    double x = camToTarget.getX();
                    double y = camToTarget.getY();
                    return Math.sqrt(x * x + y * y);
                }
            }
        }
        return Double.NaN;
    }

    @Override
    public void periodic() {
        var result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget best = result.getBestTarget();
            SmartDashboard.putNumber("Vision/BestTagID",  best.getFiducialId());
            SmartDashboard.putNumber("Vision/Yaw (deg)",  best.getYaw());
            Transform3d c2t = best.getBestCameraToTarget();
            double dist = Math.sqrt(c2t.getX() * c2t.getX() + c2t.getY() * c2t.getY());
            SmartDashboard.putNumber("Vision/Distance (m)", dist);
        } else {
            SmartDashboard.putNumber("Vision/BestTagID",  -1);
            SmartDashboard.putNumber("Vision/Distance (m)", -1);
        }
    }
}
