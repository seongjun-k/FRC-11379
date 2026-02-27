package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

/**
 * 컨베이어 벨트 서브시스템
 * 모터: CIM → SparkMax, CAN ID 10
 */
public class ConveyorSubsystem extends SubsystemBase {

    private final SparkMax m_conveyorMotor;

    public ConveyorSubsystem() {
        m_conveyorMotor = new SparkMax(ConveyorConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit((int) ConveyorConstants.CONVEYOR_CURRENT_LIMIT_AMPS);
        config.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        m_conveyorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * 컨베이어를 구동합니다.
     * @param speed -1.0 ~ 1.0 (양수 = 슈터 방향)
     */
    public void setSpeed(double speed) {
        m_conveyorMotor.set(speed);
    }

    /** 컨베이어 정지 */
    public void stop() {
        m_conveyorMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // 필요 시 SmartDashboard 디버그 추가
    }
}
