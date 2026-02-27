package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;        // ← 추가
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;    // ← 추가

public class RobotContainer {

    private final DriveSubsystem        m_drive      = new DriveSubsystem();
    private final ShooterSubsystem      m_shooter    = new ShooterSubsystem(); // ← 추가
    private final CommandXboxController m_controller = new CommandXboxController(DRIVER_CONTROLLER_PORT);

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
        // 드라이브 (기존 유지)
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -m_controller.getLeftY(),
                () -> -m_controller.getRightX()
            )
        );

        // 슈터: RT = 정방향 발사, LT = 역방향, 아날로그 0~100%
        m_shooter.setDefaultCommand(
            new ShooterCommand(
                m_shooter,
                m_controller::getRightTriggerAxis,  // RT (0.0 ~ 1.0)
                m_controller::getLeftTriggerAxis     // LT (0.0 ~ 1.0)
            )
        );
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void resetGyro() {
        m_drive.zeroGyro();
    }
}