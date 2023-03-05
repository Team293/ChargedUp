package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.subsystems.Claw;


public class MoveClaw extends CommandBase {
    public boolean opened = false;
    private final Claw m_claw;
    public final XboxController m_operatorXboxController;
    public double m_triggerDeadband = 0.05;
    public double m_velocityLimitPercentage = 0.01; //TODO


    public MoveClaw(Claw givenClaw, XboxController givenController) {
        m_claw = givenClaw;
        m_operatorXboxController = givenController;

        addRequirements(m_claw);
        SmartDashboard.putNumber("Arm Trigger Deadband", m_triggerDeadband);
        SmartDashboard.putNumber("Max Velocity Percentage", m_velocityLimitPercentage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightTrigger = m_operatorXboxController.getRightTriggerAxis(); 
        double leftTrigger = m_operatorXboxController.getLeftTriggerAxis();

        m_triggerDeadband = SmartDashboard.getNumber("Arm Trigger Deadband", m_triggerDeadband);
        m_triggerDeadband = MathUtil.clamp(m_triggerDeadband, 0.0, 1.0);

        m_velocityLimitPercentage = SmartDashboard.getNumber("Claw Velocity Limit Percentage", m_velocityLimitPercentage);
        m_velocityLimitPercentage = MathUtil.clamp(m_velocityLimitPercentage, -1.0, 1.0);

        rightTrigger = SPIKE293Utils.applyDeadband(rightTrigger, m_triggerDeadband);
        leftTrigger = SPIKE293Utils.applyDeadband(leftTrigger, m_triggerDeadband);

        rightTrigger *= m_velocityLimitPercentage; 
        leftTrigger *= m_velocityLimitPercentage;

        if (rightTrigger > leftTrigger) {
            m_claw.moveClaw(rightTrigger);
        } else {
            m_claw.moveClaw(leftTrigger);
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
