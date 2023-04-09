package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class BumpDrive extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final double m_bumpPercent;

    /**
     * Uses the bumpers on the driver controller to turn the robot small amounts.
     * This is used because it allows more accuracy when making small movements, as
     * opposed to using the joystick.
     * 
     * @param subsystem
     * @param bumpPercent
     */
    public BumpDrive(Drivetrain subsystem, double bumpPercent) {
        m_drivetrain = subsystem;
        m_bumpPercent = bumpPercent;

        addRequirements(m_drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turning;

        turning = MathUtil.clamp(m_bumpPercent, -1.0d, 1.0d);
        m_drivetrain.arcadeDrive(0.0d, turning);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }
}
