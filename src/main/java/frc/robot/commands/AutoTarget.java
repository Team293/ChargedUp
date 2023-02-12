package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.DEFAULT_FORZA_DEADBAND;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;

public class AutoTarget extends CommandBase {
  private final Targeting m_targeting;
  private final Drivetrain m_drive;
  private PIDController m_turningPID;
  private PIDController m_speedPID;
  private Double m_yaw; 
  private double m_targetArea;
  private double m_turningOutput = 0.0d;
  private double m_speedOutput = 0.5d;
  private double m_error = 0.0d;
  private double m_lastError = 0.0d;
  private double m_change = 0.0d;
  private double m_errorIntegral = 0.0d;
  // start (gives throttle) (may make it overshoot if too high)
  private final double m_P = 0.000005d;
  // finicky (depends on situation) (within 5 to 3 degress of error)
  private final double m_I = 0.00d;
  // good rule of thumb for d: m_d = m_p * 10
  private final double m_D = m_P * 10;
  public AutoTarget(Targeting targeting,Drivetrain drive) {
    m_targeting = targeting;
    m_drive = drive;
    addRequirements(targeting, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (false == m_targeting.hasTarget()) return;
    // m_yaw = m_targeting.getYaw();
    // m_targetArea = m_targeting.getArea();

    // SmartDashboard.putNumber("Yaw", m_yaw);
    
    // m_error = m_yaw;
    
    // m_change = m_error - m_lastError;
    
    // if(Math.abs(m_errorIntegral) < 5) // this is an integral limit to keep from excessive I commanded movement 
    // {
    //     //Accumulate the error into the integral
    //     m_errorIntegral += m_error;
    // }
    
    
    // m_turningOutput = (m_P * m_error) + (m_I * m_errorIntegral) + (m_D * m_change);
    m_turningOutput = m_targeting.navToTarget();
    m_speedOutput = 0.5;
    if(m_targeting.hasTarget()){
      m_speedOutput = 0.5;
    }
    else{
      m_speedOutput = 0.0;
    }

    // m_speedPID = new PIDController(0.001, 0, 0.01);
    // m_speedPID.calculate(measurement)
    // m_speedPID.

    
    
    m_speedOutput *= 1;
    m_turningOutput *= 1;

    arcadeDrive(m_speedOutput , m_turningOutput);
  }

  private void arcadeDrive(double velocity, double turning) {
    // Convert turning and speed to left right encoder velocity
    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(velocity), Math.abs(turning)), velocity);
    if (velocity >= 0.0) {
        // First quadrant, else second quadrant
        if (turning >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = velocity - turning;
        } else {
            leftMotorOutput = velocity + turning;
            rightMotorOutput = maxInput;
        }
    } else {
        // Third quadrant, else fourth quadrant
        if (turning >= 0.0) {
            leftMotorOutput = velocity + turning;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = velocity - turning;
        }
    }

    // Convert to encoder velocity
    // double leftMotorSpeed =
    // SPIKE293Utils.percentageToControllerVelocity(leftMotorOutput);
    // Right needs to be inverted
    // double rightMotorSpeed =
    // SPIKE293Utils.percentageToControllerVelocity(rightMotorOutput *-1.0d);

    // Send to motors
    m_drive.percentDrive(leftMotorOutput, rightMotorOutput);
  }

  private void pid(double error){
    double change;
    
    change = error - m_lastError;
    
    if(Math.abs(m_errorIntegral) < 5) // this is an integral limit to keep from excessive I commanded movement 
    {
        //Accumulate the error into the integral
        m_errorIntegral += m_error;
    }
    
    m_turningOutput = (m_P * m_error) + (m_I * m_errorIntegral) + (m_D * m_change);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_targeting.getTargetArea() >= 10.0;
  }
}