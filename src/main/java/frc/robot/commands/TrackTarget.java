package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;

public class TrackTarget extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final Targeting m_targeting;

    public TrackTarget(Drivetrain drivetrain, Targeting targeting) {
        m_drivetrain = drivetrain;
        m_targeting = targeting;
        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_targeting.controlLight(true);
        m_drivetrain.rotateDegrees(m_targeting.getAngleToTargetDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns false when the command should end.
    @Override
    public boolean isFinished() {
        return m_targeting.isTargeted();
    }
}
