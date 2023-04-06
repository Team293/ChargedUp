package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.classes.SmoothControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;

public class DriveToAT extends CommandBase {
    public static final double WITHIN_RANGE_MODIFIER = 1.0d/4.0d;

    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private Position2D m_targetPose;
    private SmoothControl m_smoothControl;
    private Targeting m_targeting;
    private double m_maxVelocity;
    private boolean m_inReverse = false;
    private boolean m_isDone = false;

    public DriveToAT(Position2D targetPose, double maxVelocity, boolean inReverse, Kinematics kinematics,
            Drivetrain drivetrain, Targeting targeting) {
        addRequirements(drivetrain);
        m_targetPose = targetPose;
        m_maxVelocity = maxVelocity;
        m_inReverse = inReverse;
        m_kinematics = kinematics;
        m_drivetrain = drivetrain;
        m_targeting = targeting;
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
        final double trackWidthFeet = Drivetrain.TRACK_WIDTH_FEET;
        double omegaDesired = 0.0d;
        Position2D currentPose = new Position2D(m_kinematics.getPose().getX(), m_kinematics.getPose().getY(), m_kinematics.getPose().getHeadingRadians());

        if (m_targeting.hasTarget()) {

            // if limelight has a target, update the robot pose with april tag info
            m_kinematics.setPose(m_targeting.getRobotPose());
        }

        // Have we reached the target?
        if ((trackWidthFeet * WITHIN_RANGE_MODIFIER >= m_smoothControl.getRange(m_targetPose, currentPose))
        && (Math.abs(m_kinematics.getPose().getHeadingDegrees() - m_targetPose.getHeadingDegrees()) < 5)) {
            // ending the command to allow the next sequential command with next point to run
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
        SmartDashboard.putString("Target Pose", m_targetPose.getX() + ", " + m_targetPose.getY() + ", " + m_targetPose.getHeadingDegrees());
        SmartDashboard.putString("Current Pose", currentPose.getX() + ", " + currentPose.getY() + ", " + currentPose.getHeadingDegrees());        
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