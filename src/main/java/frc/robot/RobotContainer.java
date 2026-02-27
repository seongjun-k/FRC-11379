package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakePivotCommand.Direction;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    // ── 서브시스템 ──────────────────────────────────────────────
    private final DriveSubsystem       m_drive         = new DriveSubsystem();
    private final ShooterSubsystem     m_shooter       = new ShooterSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller = new IntakeRollerSubsystem();
    private final IntakePivotSubsystem  m_intakePivot  = new IntakePivotSubsystem();
    private final ConveyorSubsystem     m_conveyor     = new ConveyorSubsystem();

    // ── 컨트롤러 ────────────────────────────────────────────────
    /** 포트 0: 드라이버 (드라이브 + 슈터) */
    private final CommandXboxController m_driverController   =
        new CommandXboxController(DRIVER_CONTROLLER_PORT);

    /**
     * 포트 1: 오퍼레이터 (인테이크 / 피벗 / 컨베이어)
     * 버튼 배치:
     *   A 버튼  (button 1) → 인테이크 롤러 구동
     *   LB 버튼 (button 5) → 피벗 UP
     *   RB 버튼 (button 6) → 피벗 DOWN
     *   B 버튼  (button 2) → 컨베이어 구동
     *
     * Xbox 버튼 번호 참고:
     *   1=A, 2=B, 3=X, 4=Y, 5=LB, 6=RB, 7=Back, 8=Start
     */
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        configureBindings();

        try {
            m_autoChooser = AutoBuilder.buildAutoChooser();
        } catch (RuntimeException e) {
            DriverStation.reportError(
                "AutoBuilder not configured, using empty auto chooser",
                e.getStackTrace()
            );
            m_autoChooser = new SendableChooser<>();
        }

        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    private void configureBindings() {

        // ── 드라이브 (기존 유지) ──────────────────────────────────
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()
            )
        );

        // ── 슈터: RT = 정방향, LT = 역방향 (기존 유지) ───────────
        m_shooter.setDefaultCommand(
            new ShooterCommand(
                m_shooter,
                m_driverController::getRightTriggerAxis,
                m_driverController::getLeftTriggerAxis
            )
        );

        // ── 인테이크 롤러: A 버튼 누르는 동안 구동 ───────────────
        // 방향 반전은 Constants.IntakeConstants.ROLLER_INVERTED 참조
        m_operatorController.a().whileTrue(new IntakeRollerCommand(m_intakeRoller));

        // ── 인테이크 피벗: LB = UP / RB = DOWN ───────────────────
        m_operatorController.leftBumper().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.UP)
        );
        m_operatorController.rightBumper().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.DOWN)
        );

        // ── 컨베이어: B 버튼 누르는 동안 구동 ────────────────────
        // 방향 반전은 Constants.ConveyorConstants.CONVEYOR_INVERTED 참조
        m_operatorController.b().whileTrue(new ConveyorCommand(m_conveyor));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void resetGyro() {
        m_drive.zeroGyro();
    }
}
