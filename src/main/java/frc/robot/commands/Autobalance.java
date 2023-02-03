package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double m_P = 0.0055d;
    // finicky (depends on situation) (within 5 to 3 degress of error)
    private double m_I = 0.00d;
    // good rule of thumb for d: m_d = m_p * 10
    private double m_D = m_P * 10;
    private double m_clamp = 0.1;
    private int m_balencedTimes = 0;
    // speed
    private Drivetrain m_driveTrain;

    public Autobalance(
            Drivetrain driveTrain) {
        m_driveTrain = driveTrain;
        addRequirements(driveTrain);
        SmartDashboard.putNumber("m_P", m_P);
        SmartDashboard.putNumber("m_I", m_I);
        SmartDashboard.putNumber("m_D", m_D);
        SmartDashboard.putNumber("m_Clamp", m_clamp);
    }

    @Override
    public void execute() {
        // calculate pid variables (error (p), change (d), error integral (i))
        m_lastError = m_error;

        m_pitch = m_driveTrain.getGyroPitchDegrees();

        m_error = m_pitch - 90;

        if (Math.abs(m_error)<3) {
            m_balencedTimes+=1;
            return;
        } else {
            m_balencedTimes = 0;
        }

        m_change = m_error - m_lastError;

        if (Math.abs(m_errorIntegral) < 5) // this is an integral limit to keep from excessive I commanded movement
        {
            // Accumulate the error into the integral
            m_errorIntegral += m_error;
        }
        SmartDashboard.putNumber("Error", m_error);
        SmartDashboard.putNumber("Change", m_change);
        SmartDashboard.putNumber("ErrorIntegral", m_errorIntegral);
        // calculate turning output using the pid
        m_velOutput = (m_P * m_error) + (m_I * m_errorIntegral) + (m_D * m_change);
        MathUtil.clamp(m_velOutput, -m_clamp, m_clamp);
        SmartDashboard.putNumber("velOutput", m_velOutput);
        m_driveTrain.arcadeDrive(m_velOutput, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return m_balencedTimes > 7 && false;
    }
}
