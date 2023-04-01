package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ProximitySensor;

public class ProximityDrive extends CommandBase {
    // All range and distance values are in inches
    private static final double TARGET_DISTANCE = 24.0d;
    private static final double SLOWDOWN_RANGE = 30.0d;
    private static final double FINISH_RANGE = 3.0d;

    private final ProximitySensor m_sensor;
    private double m_distance;
    private double m_range;
    private Drivetrain m_drivetrain;

    public ProximityDrive(Drivetrain givenDrivetrain, ProximitySensor givenSensor) {
        m_drivetrain = givenDrivetrain;
        m_sensor = givenSensor;
        m_distance = 0.0d;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_distance = m_sensor.getDistanceInches();

        m_range = m_distance - TARGET_DISTANCE;

        double percentPower = Math.min(1.0d, Math.sqrt(Math.abs(m_range)/SLOWDOWN_RANGE));

        if (m_range > 0) {
            m_drivetrain.velocityDrive(percentPower, percentPower);
        } else {
            m_drivetrain.velocityDrive(-percentPower, -percentPower);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_range) < FINISH_RANGE;
    }
}
