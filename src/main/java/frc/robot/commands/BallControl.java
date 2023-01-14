package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class BallControl extends CommandBase {

    Feeder m_feeder;

    public BallControl(Feeder feeder) {
        m_feeder = feeder;
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Assume both motors will be on!
        boolean triggerMotorOn = true;
        boolean beltMotorOn = true;

        if (true == m_feeder.isTriggerSensorBallPresent()) {
            // A ball is in the trigger position
            // Stop the trigger motor to hold it in place!
            triggerMotorOn = false;
            if (true == m_feeder.isBeltSensorBallPresent()) {
                // A ball is in the belt position
                // Stop the belt motor to hold it in place!
                beltMotorOn = false;
            }
        }

        // Enable / disable motors
        if (true == triggerMotorOn) {
            m_feeder.setTriggerMotor(0.17);
        } else {
            m_feeder.setTriggerMotor(0);
        }
        if (true == beltMotorOn) {
            m_feeder.setBeltMotor(0.70);
        } else {
            m_feeder.setBeltMotor(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
