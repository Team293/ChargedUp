package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToBalance extends CommandBase{
    private Drivetrain m_drivetrain;
    private static final double WAIT_TIME = 5000;
    private double m_startTime;

    public DriveToBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        
        m_startTime = System.currentTimeMillis();
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.percentDrive(0.3, 0.3);
    }

    @Override
    public boolean isFinished() {
        return (m_drivetrain.getGyroPitchDegrees() > 85) || ((System.currentTimeMillis() - m_startTime) >= WAIT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
