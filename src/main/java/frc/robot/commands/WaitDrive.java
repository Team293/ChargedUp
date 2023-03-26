package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class WaitDrive extends CommandBase {
    private double m_waitTime;
    private double m_startTime;
    private Drivetrain m_drivetrain;

    public WaitDrive(Drivetrain drivetrain, double waitTime) {
        m_waitTime = waitTime;

        m_waitTime *= 1000;
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.percentDrive(0.15, 0.15);
    }

    @Override
    public boolean isFinished() {
        return ((System.currentTimeMillis() - m_startTime) >= m_waitTime);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
