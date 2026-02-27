package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * 인테이크 피벗(각도 조절) 서브시스템
 * 모터: CIM → SparkMax, CAN ID 9
 */
public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax m_pivotMotor;

    public IntakePivotSubsystem() {
        m_pivotMotor = new SparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit((int) IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS);
        // 브레이크 모드: 버튼을 뗐을 때 피벗이 중력으로 처지는 것을 방지
        config.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * 피벗을 위로 올립니다 (정회전 방향).
     * 실제 기구 방향에 따라 부호를 반전하려면 PIVOT_SPEED 값의 부호를 바꿔주세요
     * → Constants.IntakeConstants.PIVOT_SPEED 참조
     */
    public void pivotUp() {
        m_pivotMotor.set(IntakeConstants.PIVOT_SPEED);
    }

    /**
     * 피벗을 아래로 내립니다 (역회전 방향).
     */
    public void pivotDown() {
        m_pivotMotor.set(-IntakeConstants.PIVOT_SPEED);
    }

    /** 피벗 정지 */
    public void stop() {
        m_pivotMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // 필요 시 SmartDashboard 디버그 추가
    }
}
