package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem m_shooter;
    private final DoubleSupplier   m_rtSupplier;
    private final DoubleSupplier   m_ltSupplier;

    public ShooterCommand(ShooterSubsystem shooter,
                          DoubleSupplier rt,
                          DoubleSupplier lt) {
        m_shooter    = shooter;
        m_rtSupplier = rt;
        m_ltSupplier = lt;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double rt = m_rtSupplier.getAsDouble();
        double lt = m_ltSupplier.getAsDouble();

        double output = rt - lt;

        if (Math.abs(output) < 0.05) output = 0.0;

        m_shooter.setOutput(output);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
