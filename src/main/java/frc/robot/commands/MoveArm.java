package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    public enum Node {
        LOW,
        MID,
        HIGH,
        SUBSTATION
    }
    private final Arm m_arm;
    public final XboxController m_operatorXboxController;
    private final Node m_node;

    public MoveArm(Arm givenArm, XboxController givenController, Node node) {
        m_arm = givenArm;
        m_operatorXboxController = givenController;
        m_node = node;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch(m_node) {
            case LOW:
                m_arm.rotateTo(m_arm.SCORE_LOW_ANGLE);
                m_arm.extendTo(m_arm.SCORE_LOW_X_INCHES);
                break;
            case MID:
                m_arm.rotateTo(m_arm.SCORE_MID_ANGLE);
                m_arm.extendTo(m_arm.SCORE_MID_X_INCHES);
                break;
            case HIGH:
                m_arm.rotateTo(m_arm.SCORE_HIGH_ANGLE);
                m_arm.extendTo(m_arm.SCORE_HIGH_X_INCHES);
                break;
            case SUBSTATION:
                m_arm.rotateTo(m_arm.SUBSTATION_PICKUP_ANGLE);
                m_arm.extendTo(m_arm.SUBSTATION_PICKUP_X_INCHES);
                break;
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

