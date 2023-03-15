package frc.robot.commands;

// import edu.wpi.first.hal.simulation.SpiReadAutoReceiveBufferCallback;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.subsystems.Arm;

public class AdjustArm extends CommandBase {

    private static final double DEFAULT_ADJUST_ARM_DEADBAND = 0.05;

    private final Arm m_arm;
    public final XboxController m_operatorXboxController;

    public double m_adjustArmDeadband;

    public AdjustArm(Arm givenArm, XboxController givenController) {
        m_arm = givenArm;
        m_operatorXboxController = givenController;

        m_adjustArmDeadband = DEFAULT_ADJUST_ARM_DEADBAND;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Arm Deadband", m_adjustArmDeadband);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Grabs joystick values
        double anglePercent = m_operatorXboxController.getLeftY();
        double extendPercent = m_operatorXboxController.getRightY();
        
        // Grabs Deadband value from SmartDashboard
        m_adjustArmDeadband = SmartDashboard.getNumber("Arm Deadband", m_adjustArmDeadband);

        // Grabs angle and extend from SPIKE293Utils
        anglePercent = SPIKE293Utils.applyDeadband(anglePercent, m_adjustArmDeadband);
        extendPercent = SPIKE293Utils.applyDeadband(extendPercent, m_adjustArmDeadband);

        //Fix the sticks being inverted
        anglePercent *= -1.0d;
        extendPercent *= -1.0d;

        // Moves the arm
        m_arm.adjustPosition(anglePercent, extendPercent);
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
