package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightInterface;

public class AutoAlign extends CommandBase {
    private double m_linearP;
    private double m_linearI;
    private double m_linearD;
    private double m_angularP;
    private double m_angularI;
    private double m_angularD;
    private double m_angularErrorSum;
    private double m_linearErrorSum;
    private double m_lastAngularError;
    private double m_lastLinearError;
    private LimelightInterface m_limelight;
    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;

    public AutoAlign(LimelightInterface limelight, Drivetrain drivetrain, Kinematics kinematics) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        m_kinematics = kinematics;
        addRequirements(limelight, drivetrain);

        m_linearP = 0.1d;
        m_linearI = 0.0d;
        m_linearD = 5d;
        m_angularP = 0.5d;
        m_angularI = 0.0d;
        m_angularD = 0.5d;
    }

    @Override
    public void execute() {
        
        /*// double norm = Math.sqrt(currentPose.getX() * currentPose.getX() + currentPose.getY() * currentPose.getY());
        double linearError = m_limelight.getDistance();
        double angularErrorDelta = angularError - m_lastAngularError;
        double linearErrorDelta = linearError - m_lastLinearError;
        m_angularErrorSum += angularError;
        m_linearErrorSum += linearError;
        double angularCommand = (m_angularP * angularError) + (m_angularI * m_angularErrorSum) + (m_angularD * angularErrorDelta);
        double linearCommand = (m_linearP * linearError) + (m_linearI * m_linearErrorSum) + (m_linearD * linearErrorDelta);
        Drivetrain.getTab().setDouble("Angular command", angularCommand);
        Drivetrain.getTab().setDouble("Linear Command", linearCommand);
        Drivetrain.getTab().setDouble("Error", linearError);
        m_lastAngularError = angularError;
        m_lastLinearError = linearError;
        // Make the drivetrain drive towards the target
        m_drivetrain.arcadeDrive(linearCommand, 0);


        double linearError = m_limelight.calcDistance();
        double angularError = */
    }

    @Override
    public boolean isFinished() {
        return m_limelight.tooClose() && m_limelight.isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }
}
