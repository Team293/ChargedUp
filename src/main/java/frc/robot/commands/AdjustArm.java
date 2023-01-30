package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AdjustArm extends CommandBase {

    private final Arm arm;
    public final XboxController m_operatorXboxController;    

    public AdjustArm(Arm givenArm, XboxController givenController) {
        arm = givenArm;
        m_operatorXboxController = givenController;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.adjustPosition(m_operatorXboxController.getLeftY(), m_operatorXboxController.getRightY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override   
    public boolean isFinished() {
        return false;
    }
}
