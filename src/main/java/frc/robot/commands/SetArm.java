package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
    private final Arm m_arm;
    private final double m_theta;
    private final double m_rInches;

    private static final int ENCODER_THRESHOLD = 3000;

    /**
     * Creates a new SetArm. This command will move the arm to the given position.
     * Really only used during autonomous. This command waits until the arm is in
     * the correct position before finishing.
     * 
     * @param givenArm The arm subsystem
     * @param theta    The angle to set the arm to
     * @param rInches  The distance from the center of the robot to the arm
     */
    public SetArm(Arm givenArm, double theta, double rInches) {
        m_arm = givenArm;
        m_theta = theta;
        m_rInches = rInches;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setPosition(m_theta, m_rInches);
    }

    @Override
    public boolean isFinished() {
        double currentPos = m_arm.getPivotEncoderUnits();
        double currentExtension = m_arm.getExtenderEncoderUnits();
        boolean movingArm = (currentPos < m_arm.getCommandedPivotEncoderPosition() + ENCODER_THRESHOLD) &&
                (currentPos > m_arm.getCommandedPivotEncoderPosition() - ENCODER_THRESHOLD);
        boolean extendingArm = (currentExtension < m_arm.getCommandedExtenderEncoderPosition() + ENCODER_THRESHOLD) &&
                (currentExtension > m_arm.getCommandedExtenderEncoderPosition() - ENCODER_THRESHOLD);
        Arm.getTab().setDouble("Current Arm Encoder Pos", currentPos);
        Arm.getTab().setDouble("Arm Target Pos", m_arm.getCommandedPivotEncoderPosition());
        Arm.getTab().setBoolean("Moving Arm", !movingArm);
        Arm.getTab().setBoolean("Extending Arm", !extendingArm);
        return movingArm && extendingArm;
    }
}
