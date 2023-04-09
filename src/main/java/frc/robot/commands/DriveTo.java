package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.classes.SmoothControl;
import frc.robot.subsystems.Drivetrain;

public class DriveTo extends CommandBase {
    public static final double WITHIN_RANGE_MODIFIER = 1.0d / 4.0d;

    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private Position2D m_targetPose;
    private SmoothControl m_smoothControl;

    private double m_maxVelocity;
    private boolean m_inReverse = false;
    private boolean m_isDone = false;

    /**
     * Drive to a target pose at a max velocity. This takes in a target pose and a
     * max velocity, as well as the kinematics of the robot and the drivetrain to
     * control. If driving backwards, the max velocity should be negative.
     * 
     * @param targetPose  The target pose to drive to
     * @param maxVelocity The max velocity to drive at (in feet per second)
     * @param kinematics  The kinematics of the robot
     * @param drivetrain  The drivetrain to control
     */
    public DriveTo(Position2D targetPose, double maxVelocity, Kinematics kinematics, Drivetrain drivetrain) {
        m_targetPose = targetPose;
        m_maxVelocity = maxVelocity;
        if (maxVelocity < 0.0d) {
            m_inReverse = true;
        }
        m_kinematics = kinematics;
        m_drivetrain = drivetrain;

        // This constructs smooth control
        m_smoothControl = new SmoothControl();
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        final double trackWidthFeet = Drivetrain.TRACK_WIDTH_FEET;
        double vR = 0.0;
        double vL = 0.0;
        double omegaDesired = 0.0d;
        Position2D currentPose = new Position2D(m_kinematics.getPose().getX(), m_kinematics.getPose().getY(),
                m_kinematics.getPose().getHeadingRadians());

        // Have we reached the target?
        if ((trackWidthFeet * WITHIN_RANGE_MODIFIER) >= m_smoothControl.getRange(m_targetPose, currentPose)) {
            // ending the command to allow the next sequential command with next point to
            // run
            m_isDone = true;
        } else {
            // Compute turn rate in radians and update range
            omegaDesired = m_smoothControl.computeTurnRate(currentPose, m_targetPose, m_maxVelocity, m_inReverse);

            // Calculate vR in feet per second
            vR = m_maxVelocity + (trackWidthFeet / 2) * omegaDesired;
            // Calculate vL in feet per second
            vL = m_maxVelocity - (trackWidthFeet / 2) * omegaDesired;

            // Converting ft/s equation output to controller velocity
            vR = SPIKE293Utils.feetPerSecToControllerVelocity(vR);
            vL = SPIKE293Utils.feetPerSecToControllerVelocity(vL);

            // Send vR and vL to velocity drive, units are in controller velocity
            m_drivetrain.velocityDrive(vL, vR);
        }

        SmartDashboard.putNumber("Desired Left Velocity (ft/s)", vL);
        SmartDashboard.putNumber("Desired Right Velocity (ft/s)", vR);
        SmartDashboard.putNumber("Auto Range", m_smoothControl.getRange(m_targetPose, currentPose));
        SmartDashboard.putNumber("Auto Omega Desired (Degrees)", Math.toDegrees(omegaDesired));
        SmartDashboard.putString("Target Pose",
                m_targetPose.getX() + ", " + m_targetPose.getY() + ", " + m_targetPose.getHeadingDegrees());
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }

    @Override
    public void end(boolean interrupted) {
        // Consider making an abstracted DrivetrainCommandBase class that has a stop()
        // that automatically stops the drivetrain when the command ends.
        // This code is repeated in many commands.
        m_drivetrain.stop();
    }
}
