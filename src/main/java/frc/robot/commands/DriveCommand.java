package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    private final DriveSubsystem  m_drive;
    private final DoubleSupplier  m_xSpeed;
    private final DoubleSupplier  m_zRotation;

    public DriveCommand(
        DriveSubsystem drive,
        DoubleSupplier xSpeed,
        DoubleSupplier zRotation
    ) {
        m_drive      = drive;
        m_xSpeed     = xSpeed;
        m_zRotation  = zRotation;
        addRequirements(m_drive); // Subsystem 점유 선언
    }

    @Override
    public void execute() {
        // DoubleSupplier로 매 루프(~20ms)마다 최신 조이스틱 값을 읽음 [file:3]
        m_drive.arcadeDrive(
            m_xSpeed.getAsDouble(),
            m_zRotation.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false; // 텔레옵 중 계속 실행
    }
}
