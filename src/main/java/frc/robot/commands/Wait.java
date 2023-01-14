package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double m_waitTime;
    private double m_startTime;

    public Wait(double waitTime) {
        m_waitTime = waitTime;
        
        m_waitTime *= 1000;
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return ((System.currentTimeMillis() - m_startTime) >= m_waitTime);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
