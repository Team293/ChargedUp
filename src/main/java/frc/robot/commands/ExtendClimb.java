package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ExtendClimb extends CommandBase {

    private final Climb m_climb;
    
    public ExtendClimb(Climb subsystem) {
        m_climb = subsystem;
        addRequirements(m_climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        m_climb.climberUp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() 
    {
        return false;
    }
}
