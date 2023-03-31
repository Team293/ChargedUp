package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    public final double HIGH_X = 45.42d;
    public final double HIGH_Y = 15.4d;
    public final double HIGH_R_INCHES = Math.sqrt(Math.pow(HIGH_X, 2.0d) + Math.pow(HIGH_Y, 2.0d));
    public final double HIGH_ANGLE = Math.atan2(HIGH_Y, HIGH_X);

    public final double MID_X = 35.67d;
    public final double MID_Y = 3.4d;
    public final double MID_R_INCHES = Math.sqrt(Math.pow(MID_X, 2.0d) + Math.pow(MID_Y, 2.0d));
    public final double MID_ANGLE = Math.atan2(MID_Y, MID_X);

    public final double HYBRID_X = 14.67d;
    public final double HYBRID_Y = -30.6d;
    public final double HYBRID_R_INCHES = Math.sqrt(Math.pow(HYBRID_X, 2.0d) + Math.pow(HYBRID_Y, 2.0d));
    public final double HYBRID_ANGLE = Math.atan2(HYBRID_Y, HYBRID_X);

    public final double SUBSTATION_X = 12.67d;
    public final double SUBSTATION_Y = 6.8d;
    public final double SUBSTATION_R_INCHES = Math.sqrt(Math.pow(SUBSTATION_X, 2.0d) + Math.pow(SUBSTATION_Y, 2.0d));
    public final double SUBSTATION_ANGLE = Math.atan2(SUBSTATION_Y, SUBSTATION_X);

    public enum Node {
        HYBRID,
        MID,
        HIGH,
        SUBSTATION,
        STOW
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
        switch (m_node) {
            case HYBRID:
                m_arm.setPosition(HYBRID_ANGLE, HYBRID_R_INCHES);
                break;
            case MID:
                m_arm.setPosition(MID_ANGLE, MID_R_INCHES);
                break;
            case HIGH:
                m_arm.setPosition(HIGH_ANGLE, HIGH_R_INCHES);
                break;
            case SUBSTATION:
                m_arm.setPosition(SUBSTATION_ANGLE, SUBSTATION_R_INCHES);
                break;
            case STOW:
                m_arm.stowArm();
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
