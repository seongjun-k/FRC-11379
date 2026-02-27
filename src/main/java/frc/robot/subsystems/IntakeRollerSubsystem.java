package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * 인테이크 롤러 서브시스템
 * 모터: Kraken X60 (TalonFX), CAN ID 8
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    private final TalonFX m_rollerMotor;

    public IntakeRollerSubsystem() {
        m_rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_rollerMotor.getConfigurator().apply(config);
    }

    /**
     * 롤러를 지정한 출력으로 구동합니다.
     * @param speed -1.0 ~ 1.0 (양수 = 인테이크 방향)
     */
    public void setSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    /** 롤러 정지 */
    public void stop() {
        m_rollerMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // 필요 시 SmartDashboard 디버그 추가
    }
}
