package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveToBalance extends CommandBase {
    private Drivetrain m_drivetrain;

    public DriveToBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        RobotContainer.getAutoBoard().setDouble("Balance speed", 0.2);
        RobotContainer.getAutoBoard().setDouble("Balance Degrees", 85);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = RobotContainer.getAutoBoard().getDouble("Balance speed", 0);
        speed = MathUtil.clamp(speed, 0, 0.5);
        RobotContainer.getAutoBoard().setDouble("Balance speed", speed);
        m_drivetrain.percentDrive(speed, speed);
    }

    @Override
    public boolean isFinished() {
        double gyro = RobotContainer.getAutoBoard().getDouble("Balance Degrees", 85);
        gyro = MathUtil.clamp(gyro, 80, 100);
        RobotContainer.getAutoBoard().setDouble("Balance Degrees", gyro);
        // track if the robot is on a slope
        boolean isOnBalance = m_drivetrain.getGyroPitchDegrees() > (gyro);
        RobotContainer.getAutoBoard().setBoolean("Is On Balance", isOnBalance);

        return m_drivetrain.getGyroPitchDegrees() > (gyro);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
