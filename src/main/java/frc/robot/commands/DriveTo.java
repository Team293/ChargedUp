package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.classes.SmoothControl;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.AutonomousCommandConstants.*;

public class DriveTo extends CommandBase {

    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private Position2D m_targetPose;
    private SmoothControl m_smoothControl;
    private double m_maxVelocity;
    private boolean m_inReverse = false;
    private boolean m_isDone = false;

    public DriveTo(Position2D targetPose, double maxVelocity, boolean inReverse, Kinematics kinematics, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        m_targetPose = targetPose;
        m_maxVelocity = maxVelocity;
        m_inReverse = inReverse;
        m_kinematics = kinematics;
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        // Content to be run when initializing the command
        // Initialize smooth control
        m_smoothControl = new SmoothControl();
    }

    @Override
    public void execute() {
        double vR = 0.0;
        double vL = 0.0;

        // Start auto nav drive routine
        if (true == m_inReverse) {
            // We're in reverse, heading needs to be reversed for smooth control algorithm
            m_targetPose.setHeadingRadians(m_targetPose.getHeadingRadians() + Math.PI);
        }

        // Compute turn rate in radians and update range
        double omegaDesired = m_smoothControl.computeTurnRate(m_kinematics.getPose(), m_targetPose, m_maxVelocity);

        if (true == m_inReverse) {
            // Calculate vR in feet per second
            vR = -m_maxVelocity - (TRACK_WIDTH_FEET / 2) * omegaDesired;
            // Calculate vL in feet per second
            vL = -m_maxVelocity + (TRACK_WIDTH_FEET / 2) * omegaDesired;
        } else {
            // Calculate vR in feet per second
            vR = m_maxVelocity + (TRACK_WIDTH_FEET / 2) * omegaDesired;
            // Calculate vL in feet per second
            vL = m_maxVelocity - (TRACK_WIDTH_FEET / 2) * omegaDesired;
        }

        SmartDashboard.putNumber("Desired Left Velocity (ft/s)", vL);
        SmartDashboard.putNumber("Desired Right Velocity (ft/s)", vR);
        SmartDashboard.putNumber("Auto Range", m_smoothControl.getRange());
        SmartDashboard.putNumber("Auto Omega Desired (Degrees)", Math.toDegrees(omegaDesired));
        SmartDashboard.putString("Next Target",
                m_targetPose.getX() + ", " + m_targetPose.getY() + ", " + m_targetPose.getHeadingDegrees());

        // Converting ft/s equation output to controller velocity
        vR = SPIKE293Utils.feetPerSecToControllerVelocity(vR);
        vL = SPIKE293Utils.feetPerSecToControllerVelocity(vL);

        // Send vR and vL to velocity drive, units are in controller velocity
        m_drivetrain.velocityDrive(vL, vR);

        // Have we reached the target?
        if (TARGET_WITHIN_RANGE_FEET >= m_smoothControl.getRange()) {
            // ending the command to allow the next sequential command with next point to
            // run
            m_isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

}
