package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Rotate extends CommandBase {

    private final Drivetrain m_drivetrain;
    private double m_targetDegrees;
    private double initalAngle;
    private double m_lastError = 0.0d;
    private double m_errorIntegral = 0.0d;
    private double m_error;
    private double m_change;
    //Base values for tuning
    private double m_vP = 0.006;
    private double m_vI = 0.000;
    private double m_vD = 0.06;


    public Rotate(Drivetrain drivetrain, double targetDegrees) {
        m_drivetrain = drivetrain;
        m_targetDegrees = targetDegrees;
        initalAngle = m_drivetrain.getGyroYawDegrees();

        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.rotateDegrees(m_targetDegrees);
        SmartDashboard.putBoolean("finished rotating", false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double percentOutput = 0.0d;
        m_error = Math.abs(m_drivetrain.getGyroYawDegrees() - Math.abs(initalAngle - m_targetDegrees));
        
        m_change = m_error - m_lastError;
        
        if(Math.abs(m_errorIntegral) < 50) // this is an integral limit to keep from excessive I commanded movement 
        {
            //Accumulate the error into the integral
            m_errorIntegral += m_error;
        }
        
        percentOutput = (m_vP * m_error) + (m_vI * m_errorIntegral) + (m_vD * m_change);
        percentOutput *= 0.4;
        clampPercentOutput(percentOutput);

        m_lastError = m_error;

        m_drivetrain.percentDrive(-percentOutput, percentOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns false when the command should end.
    @Override
    public boolean isFinished() {
        boolean retVal = false;
        if(m_error < 20){
            SmartDashboard.putBoolean("finished rotating", true);
            retVal = true;
        }
        

        return retVal;
    }
    
    public double clampPercentOutput(double percent){
        if(percent > 0.5) 
        {
            percent = 0.5;
        }
        else if(percent < -0.5) 
        {
            percent = -0.5;
        }

        return percent;
    }
}
