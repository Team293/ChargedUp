package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;


public class SetClawForTime extends CommandBase {
    private final Claw m_claw;
    private final double m_percent;
    private final Wait m_time;

    /**
     * Creates a new SetClaw, which sets the claw to a given percent speed and stops it at a given force threshold.
     * @param givenClaw The claw subsystem
     * @param percent The percent to set the claw to. -1 to close, 1 to open
     * @param power The power to stop the claw at
     */
    public SetClawForTime(Claw givenClaw, double percent, double time) {
        m_claw = givenClaw;
        m_percent = percent;
        m_time = new Wait(time);

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    /**
     * Closes or opens the claw at a given percent speed -1 to 1
     */
    @Override
    public void execute() {
        m_claw.percentClaw(m_percent, 7.0d);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Returns true when the claw is at the given power
     * @return true when the claw is feeling a certain level of resistance
     */
    @Override
    public boolean isFinished() {
        return m_time.isFinished();
    }
}
