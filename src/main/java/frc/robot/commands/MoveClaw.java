package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;


public class MoveClaw extends CommandBase {
    public boolean opened = false;
    private final Claw m_claw;
    public final XboxController m_operatorXboxController;


    public MoveClaw(Claw givenClaw, XboxController givenController) {
        m_claw = givenClaw;
        m_operatorXboxController = givenController;

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (opened) {
            m_claw.closeClaw();
            opened = false;
        } else {
            m_claw.openClaw();
            opened = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
