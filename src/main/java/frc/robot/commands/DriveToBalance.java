package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToBalance extends CommandBase {
    private Drivetrain m_drivetrain;
    private final static double WAIT_TIME = 1000.0d;
    private double m_startTime = -1;
    private boolean m_surpassedLimit = false;

    public DriveToBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.percentDrive(0.1, 0.1);
    }

    @Override
    public boolean isFinished() {
        return m_drivetrain.getGyroPitchDegrees() > (85 + 15);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
