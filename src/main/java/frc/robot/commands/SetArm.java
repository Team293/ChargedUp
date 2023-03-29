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
     * Really only used during autonomous.
     * 
     * @param givenArm The arm subsystem
     * @param theta    The angle to set the arm to
     * @param rInches  The distance from the center of the robot to the arm
     */
    public SetArm(Arm givenArm, double theta, double rInches) {
        m_arm = givenArm;
        m_theta = theta;
        m_rInches = rInches;

        // // find the difference in theta and rInches from the current position
        // // and calculate approximately how long it will take to get there
        // double deltaTheta = Math.abs(m_theta - m_arm.getTheta()) / 180.0d;
        // double deltaR = Math.abs(m_rInches - m_arm.getRInches()) / 24.0d;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setPosition(m_theta, m_rInches);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Returns true when the arm is at the given position
     * 
     * @return true when the arm is at the given position
     */
    @Override
    public boolean isFinished() {
        double currentPos = m_arm.getCurrentEncoderUnits();
        double currentExtension = m_arm.getCurrentEncoderExtension();
        boolean movingArm = (currentPos < m_arm.getCommandedEncoderPosition() + ENCODER_THRESHOLD) &&
                (currentPos > m_arm.getCommandedEncoderPosition() - ENCODER_THRESHOLD);
        boolean extendingArm = (currentExtension < m_arm.getCommandedExtentionPosition() + ENCODER_THRESHOLD) &&
                (currentExtension > m_arm.getCommandedExtentionPosition() - ENCODER_THRESHOLD);
        Arm.getTab().setDouble("Current Arm Encoder Pos", currentPos);
        Arm.getTab().setDouble("Arm Target Pos", m_arm.getCommandedEncoderPosition());
        Arm.getTab().setBoolean("Moving Arm", !movingArm);
        Arm.getTab().setBoolean("Extending Arm", !extendingArm);
        return movingArm && extendingArm;
    }
}
