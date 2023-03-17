package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase{
    
    private Claw m_claw;

    public OpenClaw(Claw claw){
        m_claw = claw;
    }

    
    @Override
    public void initialize() {
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.percentClaw(0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
