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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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
    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // ── 오도메트리 ────────────────────────────
    private final DifferentialDriveKinematics m_kinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    private final DifferentialDriveOdometry m_odometry;

    // ───────────────────────────────────────────────
    public DriveSubsystem() {
        configureTalon(m_leftFront,  false);
        configureTalon(m_leftRear,   false);
        configureTalon(m_rightFront, true);  // 우측 반전
        configureTalon(m_rightRear,  true);

        m_drive.setSafetyEnabled(false);

        m_leftRear.setControl(new Follower(LEFT_FRONT_ID,   MotorAlignmentValue.Aligned));
        m_rightRear.setControl(new Follower(RIGHT_FRONT_ID, MotorAlignmentValue.Aligned));

        m_leftFront.setPosition(0);
        m_rightFront.setPosition(0);

        m_odometry = new DifferentialDriveOdometry(
            getGyroRotation(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
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

    // ── 주기적 오도메트리 업데이트 ────────────────
    @Override
    public void periodic() {
        m_odometry.update(
            getGyroRotation(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );

        var pose = m_odometry.getPoseMeters();
        SmartDashboard.putNumber("Odometry X (m)", pose.getX());
        SmartDashboard.putNumber("Odometry Y (m)", pose.getY());
        SmartDashboard.putNumber("Odometry Deg",  pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Left Dist (m)",  getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Dist (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Gyro Yaw (deg)", getGyroRotation().getDegrees());
    }

    // ── PathPlanner 필수 메서드 ─────────────────────

    /** 현재 필드 기준 위치 반환 */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /** 자이로 0점 초기화 */
    public void zeroGyro() {
        m_gyro.reset();
    }

    /**
     * 텔레옵 직진 보정 및 AutoAlignCommand에서 사용하는 자이로 각도 반환
     * 반환값: WPILib 기준(반시계 = 양수)으로 변환된 degree
     */
    public double getGyroAngle() {
        return -m_gyro.getAngle();
    }

    /** 오토 시작점 기준으로 오도메트리 초기화 */
    public void resetPose(Pose2d pose) {
        m_leftFront.setPosition(0);
        m_rightFront.setPosition(0);
        m_odometry.resetPosition(
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
