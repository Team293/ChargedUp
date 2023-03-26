package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
    private final Arm m_arm;
    private final double m_theta;
    private final double m_rInches;

    public SetArm(Arm givenArm, double theta, double rInches) {
        m_arm = givenArm;
        m_theta = theta;
        m_rInches = rInches;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_arm.setPosition(m_theta, m_rInches);
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
