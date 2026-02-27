package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class StraightAuto extends Command {
    private final DriveSubsystem drive;
    private final double speed;
    private final double durationSec;
    private double startTime;

    public StraightAuto(DriveSubsystem drive, double speed, double durationSec) {
        this.drive = drive;
        this.speed = speed;
        this.durationSec = durationSec;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(speed, 0.0);  // 전진만
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime >= durationSec;
    }
}