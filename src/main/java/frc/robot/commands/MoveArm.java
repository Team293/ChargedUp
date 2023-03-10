package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    public final double SCORE_HYBRID_X_INCHES = 20.0d; // TODO: Low: Find the value for how far the extension motor has
                                                       // to extend in inches.
    public final double SCORE_HYBRID_ANGLE = 42.0d;
    public final double SCORE_MID_X_INCHES = 40.0d; // TODO: Mid: Find the value for how far the extension motor has to
                                                    // extend in inches.
    public final double SCORE_MID_ANGLE = 87.0d;
    public final double SCORE_HIGH_X_INCHES = 60.0; // TODO: High: Find the value for how far the extension motor has to
                                                    // extend in inches.
    public final double SCORE_HIGH_ANGLE = 106.0d;
    public final double SUBSTATION_PICKUP_ANGLE = 104.0d;
    public final double SUBSTATION_PICKUP_X_INCHES = 10.0d; // TODO: High: Find the value for how far the extension
                                                            // motor has to extend in inches.

    public enum Node {
        HYBRID,
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
            case HYBRID:
                m_arm.rotateTo(SCORE_HYBRID_ANGLE);
                m_arm.extendTo(SCORE_HYBRID_X_INCHES);
                break;
            case MID:
                m_arm.rotateTo(SCORE_MID_ANGLE);
                m_arm.extendTo(SCORE_MID_X_INCHES);
                break;
            case HIGH:
                m_arm.rotateTo(SCORE_HIGH_ANGLE);
                m_arm.extendTo(SCORE_HIGH_X_INCHES);
                break;
            case SUBSTATION:
                m_arm.rotateTo(SUBSTATION_PICKUP_ANGLE);
                m_arm.extendTo(SUBSTATION_PICKUP_X_INCHES);
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

