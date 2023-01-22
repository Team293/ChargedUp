package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Autobalance extends CommandBase {
    private Double m_pitch;
    private double m_error = 0.0d;
    private double m_lastError = 0.0d;
    private double m_change = 0.0d;
    private double m_errorIntegral = 0.0d;
    private double m_velOutput = 0.0d;
    // start (gives throttle) (may make it overshoot if too high)
    private final double m_P = 0.0015d;
    // finicky (depends on situation) (within 5 to 3 degress of error)
    private final double m_I = 0.00d;
    // good rule of thumb for d: m_d = m_p * 10
    private final double m_D = m_P * 10;
    // speed
    private final double vel = 5;
    private Drivetrain m_driveTrain;


    public Autobalance(
            Drivetrain driveTrain) {
        m_driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
                // calculate pid variables (error (p), change (d), error integral (i))
        m_lastError = m_error;

        m_pitch = m_driveTrain.getGyroPitchDegrees();

        m_error = m_pitch;

        m_change = m_error - m_lastError;

        if(Math.abs(m_errorIntegral) < 5) // this is an integral limit to keep from excessive I commanded movement 
        {
            //Accumulate the error into the integral
            m_errorIntegral += m_error;
        }
        
        // calculate turning output using the pid
        m_velOutput = (m_P * m_error) + (m_I * m_errorIntegral) + (m_D * m_change);
        m_driveTrain.arcadeDrive(m_velOutput*vel, 0);
    }
}
