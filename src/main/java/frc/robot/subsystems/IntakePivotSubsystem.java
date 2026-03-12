package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake Pivot Subsystem (Position PID + Soft Limit)
 * REVLib 2025: setReference uses 4-param overload (value, type, slot, arbFF)
 * which is NOT deprecated, replacing the 3-param deprecated version.
 */
public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax                  m_pivotMotor;
    private final RelativeEncoder           m_encoder;
    private final SparkClosedLoopController m_controller;

    private double m_targetPosition = IntakeConstants.PIVOT_HOME_POS;

    public IntakePivotSubsystem() {
        m_pivotMotor = new SparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(IntakeConstants.PIVOT_FORWARD_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(IntakeConstants.PIVOT_REVERSE_SOFT_LIMIT);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(IntakeConstants.PIVOT_KP)
            .i(IntakeConstants.PIVOT_KI)
            .d(IntakeConstants.PIVOT_KD)
            .outputRange(-0.2, 0.2);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit((int) IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS)
            .secondaryCurrentLimit(IntakeConstants.PIVOT_SECONDARY_CURRENT_LIMIT_AMPS)
            .idleMode(IdleMode.kBrake)
            .apply(softLimitConfig)
            .apply(closedLoopConfig);

        m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder    = m_pivotMotor.getEncoder();
        m_controller = m_pivotMotor.getClosedLoopController();

        m_encoder.setPosition(IntakeConstants.PIVOT_HOME_POS);
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    private double getGravityFeedforward() {
        double degreesRotated = m_encoder.getPosition() * (360.0 / IntakeConstants.PIVOT_GEAR_RATIO);
        double angleFromHorizontalDeg = IntakeConstants.PIVOT_HOME_ANGLE_DEG - degreesRotated;
        return IntakeConstants.PIVOT_KG_VOLTS * Math.cos(Math.toRadians(angleFromHorizontalDeg));
    }

    public void deployIntake() {
        m_targetPosition = IntakeConstants.PIVOT_INTAKE_POS;
    }

    public void retractIntake() {
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    public void holdCurrentPosition() {
        m_targetPosition = m_encoder.getPosition();
    }

    public void bumpDown() {
        double newTarget = m_targetPosition + IntakeConstants.PIVOT_BUMP_STEP;
        m_targetPosition = Math.min(newTarget, IntakeConstants.PIVOT_FORWARD_SOFT_LIMIT);
    }

    public void bumpUp() {
        double newTarget = m_targetPosition - IntakeConstants.PIVOT_BUMP_STEP;
        m_targetPosition = Math.max(newTarget, IntakeConstants.PIVOT_REVERSE_SOFT_LIMIT);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(m_encoder.getPosition() - m_targetPosition) < 0.3;
    }

    public boolean isOverCurrent() {
        return m_pivotMotor.getOutputCurrent() > IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS;
    }

    public void resetEncoder() {
        m_encoder.setPosition(IntakeConstants.PIVOT_HOME_POS);
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    @Override
    public void periodic() {
        // REVLib 2025 non-deprecated 4-param overload:
        // setReference(value, controlType, slot, arbFF, arbFFUnits)
        m_controller.setReference(
            m_targetPosition,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            getGravityFeedforward(),
            ArbFFUnits.kVoltage
        );

        SmartDashboard.putNumber("Pivot/Position (rot)", m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Target (rot)",   m_targetPosition);
        SmartDashboard.putNumber("Pivot/Current (A)",    m_pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/GravityFF (V)",  getGravityFeedforward());
        SmartDashboard.putBoolean("Pivot/AtTarget",      isAtTarget());
    }
}
