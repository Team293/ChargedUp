package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopTimer extends CommandBase {
    private final Supplier<Long> m_resetTime;
    

    public StopTimer(Supplier<Long> elapsedTime) {
        m_resetTime = elapsedTime;
        m_resetTime.get();
    }

    @Override
    public void initialize() {
        m_resetTime.get();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
