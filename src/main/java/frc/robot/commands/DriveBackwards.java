package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.subsystems.Drivetrain;

public class DriveBackwards extends CommandBase {
    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private double m_speed;
    private double m_distance;

    public DriveBackwards(Drivetrain drivetrain, Kinematics kinematics, double speed, double distance) {
        m_drivetrain = drivetrain;
        m_kinematics = kinematics;
        m_speed = speed;

        m_distance = distance;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.percentDrive(m_speed, m_speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_kinematics.getPose().getX()) > Math.abs(m_distance);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
