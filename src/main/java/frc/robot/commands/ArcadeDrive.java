package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    public static final double DEFAULT_FORZA_DEADBAND = 0.04;
    public static final double DEFAULT_ARCADE_JOY_DEADBAND = 0.04;
    public static final boolean DEFAULT_FORZA_MODE = true;

    public static final double DEFAULT_MAX_VELOCITY_PERCENTAGE = 0.95;
    public static final double DEFAULT_MAX_TURNING_SPEED = 0.55d;

    private final Drivetrain m_drivetrain;
    private final XboxController m_xboxcontroller;

    private double m_arcadeDeadband;
    private double m_velocityLimitPercentage;
    private double m_turningLimitPercentage;

    /**
     * Used to drive the robot using a single joystick. The joystick is used to
     * turn the robot and to move the robot forward and backward. The left stick
     * of the supplied Xbox controller is used for input. X axis is used for
     * turning, and the Y axis is used for forward/backward movement.
     *
     * @param suppliedDrivetrain The drivetrain subsystem this command will run on.
     * @param xboxcontroller     The Xbox controller to use for input.
     */
    public ArcadeDrive(Drivetrain suppliedDrivetrain, XboxController xboxcontroller) {
        m_drivetrain = suppliedDrivetrain;
        addRequirements(m_drivetrain);
        m_xboxcontroller = xboxcontroller;

        m_velocityLimitPercentage = DEFAULT_MAX_VELOCITY_PERCENTAGE;
        m_turningLimitPercentage = DEFAULT_MAX_TURNING_SPEED;
        m_arcadeDeadband = DEFAULT_ARCADE_JOY_DEADBAND;
        Drivetrain.getTab().setDouble("Arcade Joy Deadband", m_arcadeDeadband);
        Drivetrain.getTab().setDouble("Max Velocity Percentage", m_velocityLimitPercentage);
        Drivetrain.getTab().setDouble("Max Turning Percentage", m_turningLimitPercentage);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turning;
        double speed;

        // Get deadband value set in SmartDashboard
        m_arcadeDeadband = Drivetrain.getTab().getDouble("Arcade Joy Deadband", DEFAULT_ARCADE_JOY_DEADBAND);
        m_arcadeDeadband = MathUtil.clamp(m_arcadeDeadband, 0.0d, 1.0d);

        m_velocityLimitPercentage = Drivetrain.getTab().getDouble("Max Velocity Percentage",
                DEFAULT_MAX_VELOCITY_PERCENTAGE);
        m_velocityLimitPercentage = MathUtil.clamp(m_velocityLimitPercentage, 0.0d, 1.0d);
        Drivetrain.getTab().setDouble("Max Velocity Percentage", m_velocityLimitPercentage);

        m_turningLimitPercentage = Drivetrain.getTab().getDouble("Max Turning Percentage", DEFAULT_MAX_TURNING_SPEED);
        m_turningLimitPercentage = MathUtil.clamp(m_turningLimitPercentage, 0.0d, 1.0d);
        Drivetrain.getTab().setDouble("Max Turning Percentage", m_turningLimitPercentage);

        // Get turning. Note that the controls are inverted!
        turning = m_xboxcontroller.getLeftX();

        // Checks if joystick value is higher or lower than deadband value
        turning = SPIKE293Utils.applyDeadband(turning, m_arcadeDeadband);

        speed = -m_xboxcontroller.getLeftY();
        speed = SPIKE293Utils.applyDeadband(speed, m_arcadeDeadband);

        // Clamp input to verify they are valid and greater than the deadband
        turning = MathUtil.clamp(turning, -1.0d, 1.0d);
        speed = MathUtil.clamp(speed, -1.0d, 1.0d);

        // Apply limiting percentage
        turning *= m_turningLimitPercentage;
        speed *= m_velocityLimitPercentage;
        m_drivetrain.arcadeDrive(speed, turning);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
