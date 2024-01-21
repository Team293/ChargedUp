package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AutoAlign extends CommandBase {
    private static final double LINEAR_INTERGRAL_LIMIT = 5;
    private static final double ANGULAR_INTEGRAL_LIMIT = 5;
    // linear pid
    private double m_linearError;
    private double m_linearLastError;
    private double m_linearP;
    private double m_linearI;
    private double m_linearD;
    private double m_linearChange;
    private double m_linearIntegralError;
    private double m_linearClamp;
    private double m_linearVelOutput;

    // angular pid
    private double m_angularError;
    private double m_angularLastError;
    private double m_angularP;
    private double m_angularI;
    private double m_angularD;
    private double m_angularChange;
    private double m_angularIntegralError;
    private double m_angularClamp;
    private double m_angularVelOutput;

    private Limelight m_limelight;
    private final Kinematics m_kinematics;
    private final Drivetrain m_drivetrain;

    public AutoAlign(Limelight limelight, Kinematics kinematics, Drivetrain drivetrain) {
        m_limelight = limelight;
        m_kinematics = kinematics;
        m_drivetrain = drivetrain;
        m_linearI = 0.0;
        m_linearP = 0.001;
        m_linearD = 0.0;

        m_angularP = 2;
        m_angularI = 0;
        m_angularD = 0.0;
        Drivetrain.getTab().setDouble("Angular P", m_angularP);
        Drivetrain.getTab().setDouble("Angular I", m_angularI);
        Drivetrain.getTab().setDouble("Angular D", m_angularD);
    }

    @Override
    public void execute() {
        m_angularP = Drivetrain.getTab().getDouble("Angular P", m_angularP);
        m_angularI = Drivetrain.getTab().getDouble("Angular I", m_angularI);
        m_angularD = Drivetrain.getTab().getDouble("Angular D", m_angularD);

        m_linearClamp = 1d;
        m_angularClamp = 1d;
        // set last errors to the past error
        m_linearLastError = m_linearError;
        m_angularLastError = m_angularError;

        Position2D robotPosition = m_kinematics.getPose(); // Robot space
        double tagHeading = LimelightHelpers.getTargetPose_RobotSpace(m_limelight.getLimelightName())[5];
        m_angularError = LimelightHelpers.getTX("limelight"); // (tagHeading - robotPosition.getHeadingDegrees());
        m_linearError = m_limelight.getDistance();
        m_linearChange = m_linearError - m_linearLastError;
        m_angularChange = m_angularError - m_angularLastError;

        if (Math.abs(m_linearError) < LINEAR_INTERGRAL_LIMIT) {
            m_linearIntegralError += m_linearError;
        }
        if (Math.abs(m_angularError) < ANGULAR_INTEGRAL_LIMIT) {
            m_angularIntegralError += m_angularError;
        }

        m_linearVelOutput = (m_linearP * m_linearError) + (m_linearI * m_linearIntegralError) + (m_linearD * m_linearChange);
        m_linearVelOutput = MathUtil.clamp(m_linearVelOutput, -m_linearClamp, m_linearClamp);

        //  Angular align
        m_angularVelOutput = (m_angularP * m_angularError) + (m_angularI * m_angularIntegralError) + (m_angularD * m_angularChange);
        m_angularVelOutput = MathUtil.clamp(m_angularVelOutput, -m_angularClamp, m_angularClamp);

        if(Math.abs(m_angularError) < 0.1)
        {
            /* Found the target */
            m_angularVelOutput = 0.0d;
            m_angularIntegralError = 0.0d;
        }

        /* Only turn for the time being */
        m_drivetrain.arcadeDrive(0, m_angularVelOutput);


        Drivetrain.getTab().setDouble("m_angularError (DEG)", m_angularError);
        Drivetrain.getTab().setDouble("m_angularVelOutput", m_angularVelOutput);
        Drivetrain.getTab().setDouble("Robot heading (DEG)", robotPosition.getHeadingDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false; // Math.abs(m_angularError) < 1;
    }
}
