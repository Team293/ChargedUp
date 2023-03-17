package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    public final double SCORE_HYBRID_R_INCHES = 38.1365966d; /* Coordinates: 15.6in, -34.8in */
    public final double SCORE_HYBRID_ANGLE = -1.14937712d;

    public final double SCORE_MID_R_INCHES = 34.712d; /* Coordinates: 36.6in, -0.8in */
    public final double SCORE_MID_ANGLE = -1236d;

    public final double SCORE_HIGH_R_INCHES = 46.92472d; /* Coordinates: 46.35in, 11.2in */
    public final double SCORE_HIGH_ANGLE = 0.125958d;

    public final double SUBSTATION_PICKUP_ANGLE = -0.1734d; /* Coordinates: 13.6in, 2.6in */
    public final double SUBSTATION_PICKUP_X_INCHES = 34.712d;

    public enum Node {
        HYBRID,
        MID,
        HIGH,
        SUBSTATION
    }

    private final Arm m_arm;
    private final Node m_node;

    public MoveArm(Arm givenArm, Node node) {
        m_arm = givenArm;
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
                m_arm.setPosition(SCORE_HYBRID_ANGLE, SCORE_HYBRID_R_INCHES);
                break;
            case MID:
                m_arm.setPosition(SCORE_MID_ANGLE, SCORE_MID_R_INCHES);
                break;
            case HIGH:
                m_arm.setPosition(SCORE_HIGH_ANGLE, SCORE_HIGH_R_INCHES);
                break;
            case SUBSTATION:
                m_arm.setPosition(SUBSTATION_PICKUP_ANGLE, SUBSTATION_PICKUP_X_INCHES);
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

