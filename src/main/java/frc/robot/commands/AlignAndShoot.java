package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignAndShoot extends CommandBase {
    private final Limelight m_limelight;
    private final Drivetrain m_drivetrain;
    private final Kinematics m_kinematics;

    public AlignAndShoot(Limelight limelight, Drivetrain drivetrain, Kinematics kinematics) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        m_kinematics = kinematics;
        addRequirements(m_limelight, m_drivetrain);

    }

    @Override
    public void execute() {
        new AutoAlign(m_limelight, m_kinematics, m_drivetrain);
        if (m_limelight.angleAligned    ()) {
            new Intake(m_drivetrain);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}