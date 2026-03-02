package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // ── 모터 ───────────────────────────────
    private final TalonFX m_leftFront  = new TalonFX(LEFT_FRONT_ID);
    private final TalonFX m_leftRear   = new TalonFX(LEFT_REAR_ID);
    private final TalonFX m_rightFront = new TalonFX(RIGHT_FRONT_ID);
    private final TalonFX m_rightRear  = new TalonFX(RIGHT_REAR_ID);

    private final DifferentialDrive m_drive = new DifferentialDrive(
        m_leftFront::set,
        m_rightFront::set
    );

    // ── 센서 ───────────────────────────────
    private final AHRS            m_gyro;
    private final VisionSubsystem m_vision;

    // ── 칼만 필터 내장 위치 추정기 ───────────────────
    private final DifferentialDriveKinematics    m_kinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    private final DifferentialDrivePoseEstimator m_poseEstimator;

    // ───────────────────────────────────────────────
    public DriveSubsystem(VisionSubsystem vision) {
        m_gyro   = new AHRS(AHRS.NavXComType.kMXP_SPI);
        m_vision = vision;

        configureTalon(m_leftFront,  false);
        configureTalon(m_leftRear,   false);
        configureTalon(m_rightFront, true);  // 우측 반전
        configureTalon(m_rightRear,  true);

        m_drive.setSafetyEnabled(false);

        m_leftRear.setControl(new Follower(LEFT_FRONT_ID,   MotorAlignmentValue.Aligned));
        m_rightRear.setControl(new Follower(RIGHT_FRONT_ID, MotorAlignmentValue.Aligned));

        m_leftFront.setPosition(0);
        m_rightFront.setPosition(0);

        m_poseEstimator = new DifferentialDrivePoseEstimator(
            m_kinematics,
            getGyroRotation(),
            getLeftDistanceMeters(),
            getRightDistanceMeters(),
            new Pose2d(),
            // 오도메트리 신뢰도 표준편차 [x(m), y(m), theta(rad)]
            // 값이 작을수록 오도메트리를 더 신뢰 -> 주행 중 오차가 적으면 낮춰라
            VecBuilder.fill(0.05, 0.05, 0.05),
            // 비전 측정값 신뢰도 표준편차 [x(m), y(m), theta(rad)]
            // 값이 작을수록 비전을 더 신뢰 -> 태그가 멀리 있으면 크게 설정
            VecBuilder.fill(0.5, 0.5, 0.5)
        );

        configurePathPlanner();
    }

    // ── PathPlanner 설정 ──────────────────────────
    private void configurePathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPLTVController(0.02),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent()
                    && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    // ── TalonFX 공통 설정 ─────────────────────────
    private void configureTalon(TalonFX motor, boolean inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = SUPPLY_CURRENT_LIMIT_AMPS;
        motor.getConfigurator().apply(config);
    }

    /** NavX 각도를 WPILib 기준(반시계가 +)으로 변환 */
    private Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    // ── 주기적 업데이트 ───────────────────────────
    @Override
    public void periodic() {
        // 1단계: 엔코더+자이로 오도메트리 업데이트
        m_poseEstimator.update(
            getGyroRotation(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );

        // 2단계: ★ 칼만 필터 핵심 - 비전 측정값 자동 융합
        // 타임스탬프를 함께 넣어서 시간 지연 보정도 수행
        m_vision.getEstimatedPose().ifPresent(estimated ->
            m_poseEstimator.addVisionMeasurement(
                estimated.estimatedPose.toPose2d(),
                estimated.timestampSeconds
            )
        );

        var pose = m_poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Pose X (m)",    pose.getX());
        SmartDashboard.putNumber("Pose Y (m)",    pose.getY());
        SmartDashboard.putNumber("Pose Deg",      pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Left Dist (m)",  getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Dist (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Gyro Yaw (deg)", getGyroRotation().getDegrees());
    }

    // ── PathPlanner 필수 메서드 ──────────────────────

    /** 현재 필드 기준 위치 반환 (칼만 필터 적용된 추정치) */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /** 자이로 0점 초기화 */
    public void zeroGyro() {
        m_gyro.reset();
    }

    /** 텔레옥 직진 보정 및 AutoAlignCommand에서 사용 */
    public double getGyroAngle() {
        return -m_gyro.getAngle();
    }

    /** 오토 시작점 기준으로 PoseEstimator 초기화 */
    public void resetPose(Pose2d pose) {
        m_leftFront.setPosition(0);
        m_rightFront.setPosition(0);
        m_poseEstimator.resetPosition(
            getGyroRotation(),
            0, 0,
            pose
        );
    }

    /** 로봇 상대 속도 반환 (PathPlanner 요구) */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMS(),
                getRightVelocityMS()
            )
        );
    }

    /** PathPlanner에서 계산한 목표 속도로 구동 */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_SPEED_MPS);
        m_drive.tankDrive(
            wheelSpeeds.leftMetersPerSecond  / MAX_SPEED_MPS,
            wheelSpeeds.rightMetersPerSecond / MAX_SPEED_MPS
        );
    }

    // ── 텔레옥 드라이브 ─────────────────────────

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_drive.arcadeDrive(
            xSpeed    * DRIVE_SPEED_SCALE,
            zRotation * DRIVE_ROTATION_SCALE
        );
    }

    public void stopDrive() {
        m_drive.stopMotor();
    }

    // ── 내부 엔코더 헬퍼 ────────────────────────

    private double getLeftDistanceMeters() {
        return m_leftFront.getPosition().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }

    private double getRightDistanceMeters() {
        return m_rightFront.getPosition().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }

    private double getLeftVelocityMS() {
        return m_leftFront.getVelocity().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }

    private double getRightVelocityMS() {
        return m_rightFront.getVelocity().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }
}
