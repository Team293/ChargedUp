package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    public static final double HIGH_X = 45.42d;
    public static final double HIGH_Y = 15.4d;
    public static final double HIGH_R_INCHES = Math.sqrt(Math.pow(HIGH_X, 2.0d) + Math.pow(HIGH_Y, 2.0d));
    public static final double HIGH_ANGLE = Math.atan2(HIGH_Y, HIGH_X);

    public static final double MID_X = 35.67d;
    public static final double MID_Y = 3.4d;
    public static final double MID_R_INCHES = Math.sqrt(Math.pow(MID_X, 2.0d) + Math.pow(MID_Y, 2.0d));
    public static final double MID_ANGLE = Math.atan2(MID_Y, MID_X);

    public static final double HYBRID_X = 14.67d;
    public static final double HYBRID_Y = -30.6d;
    public static final double HYBRID_R_INCHES = Math.sqrt(Math.pow(HYBRID_X, 2.0d) + Math.pow(HYBRID_Y, 2.0d));
    public static final double HYBRID_ANGLE = Math.atan2(HYBRID_Y, HYBRID_X);

    public static final double SUBSTATION_X = 35.07279481d;
    public static final double SUBSTATION_Y = -1.658797242d;
    public static final double SUBSTATION_R_INCHES = Math
            .sqrt(Math.pow(SUBSTATION_X, 2.0d) + Math.pow(SUBSTATION_Y, 2.0d));
    public static final double SUBSTATION_ANGLE = Math.atan2(SUBSTATION_Y, SUBSTATION_X);

    public enum Node {
        HYBRID,
        MID,
        HIGH,
        SUBSTATION,
        STOW
    }

    private final Arm m_arm;
    private final Node m_node;

    /**
     * Moves the arm to a given node height, high mid or low, or other arbitrary
     * positions, using an enum.
     * 
     * @param givenArm
     * @param node
     */
    public MoveArm(Arm givenArm, Node node) {
        m_arm = givenArm;
        m_node = node;

        addRequirements(m_arm);
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

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Consider making a SingleCommandBase class that has a default isFinished()
        // method that returns true. This code is repeated in many commands.
        return true;
    }
}
