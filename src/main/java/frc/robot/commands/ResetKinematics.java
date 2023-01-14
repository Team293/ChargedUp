package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;

public class ResetKinematics extends CommandBase {

    private Drivetrain m_drivetrain;
    private Position2D m_resetPosition;
    private Kinematics m_kinematics;

    public ResetKinematics(Position2D startingPose, Drivetrain drivetrain, Kinematics kinematics) {
        m_drivetrain = drivetrain;
        m_resetPosition = startingPose;
        m_kinematics = kinematics;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_kinematics.setPose(m_resetPosition);
        m_drivetrain.resetGyro(m_resetPosition.getHeadingDegrees());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
