package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveToBalance extends CommandBase {
    private Drivetrain m_drivetrain;

    /**
     * This is deprecated and has been replaced with a specified distance driveTo
     * command. Please use that instead. This command drives the robot forward until
     * it reaches a certain angle. This is used to balance the robot on the charge
     * station.
     * 
     * @param drivetrain
     */
    public DriveToBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        RobotContainer.getAutoBoard().setDouble("Balance speed", 0.2);
        RobotContainer.getAutoBoard().setDouble("Balance Degrees", 85);
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
}
