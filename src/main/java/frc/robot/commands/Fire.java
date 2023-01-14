package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.WriteToCSV;
import frc.robot.subsystems.Feeder;

public class Fire extends CommandBase {

    Feeder m_feeder;
    Launcher m_launcher;
    Targeting m_targeting;
    WriteToCSV m_logger;
    int m_delayCounts;
    final boolean m_overrideTargeting;
    final double m_rpmOverrideSet;

    public Fire(Feeder feeder, Launcher launcher, Targeting targeting, WriteToCSV logger) {
        m_feeder = feeder;
        m_launcher = launcher;
        m_targeting = targeting;
        m_delayCounts = 0;
        m_logger = logger;

        //We are NOT ignoring the targeting subsystem
        m_overrideTargeting = false;
        m_rpmOverrideSet = 0.0d;

        addRequirements(m_feeder, m_launcher);
    }

    public Fire(Feeder feeder, Launcher launcher, Targeting targeting, WriteToCSV logger, double rpmOverrideSet) {
        m_feeder = feeder;
        m_launcher = launcher;
        m_targeting = targeting;
        m_delayCounts = 0;
        m_logger = logger;
        
        //We're ignoring the targeting system and manually setting the launcher RPMs
        m_overrideTargeting = true;
        m_rpmOverrideSet = rpmOverrideSet;

        addRequirements(m_feeder, m_launcher);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Assume both motors will be on!
        boolean triggerMotorOn = true;
        boolean beltMotorOn = false;
        boolean feed = false;
        double triggerMotorSet = 0.0d;
        double beltMotorSet = 0.0d;
        double rpmSet = -1.0d;
        boolean launcherReady = false;

        m_delayCounts--;

        m_targeting.controlLight(true);

        if (false == m_feeder.isTriggerSensorBallPresent()) {
            // No ball present
            if (0 >= m_delayCounts) {
                m_delayCounts = 0;
                // Attampt to load the next ball
                beltMotorOn = true;
                feed = true;
            }
        } else {
            // Ball in position to fire
            m_delayCounts = 7; // Force a wait of 350 ms before attempting to load the next ball

            // Calc launcher RPMs
            if(true == m_overrideTargeting){
                //We're ignoring the targeting module!
                rpmSet = m_rpmOverrideSet;
                //Set the launcher RPMs
                m_launcher.setRpm(rpmSet);
            } else {
                // only set the shooter to an rpm if it has a target
                // Calc launcher RPMs
                rpmSet = m_targeting.calcShooterRPM();
                m_launcher.setRpm(rpmSet);
            }

            launcherReady = m_launcher.isReady();
            if (false == launcherReady) {
                // The launcher is not ready!
                // Turn the trigger motor off!
                triggerMotorOn = false;
            }
        }

        // Enable / disable motors
        if (true == triggerMotorOn) {
            if (true == feed) {
                triggerMotorSet = 0.17d;
                // Write to file - get launcher speed, get distance
            } else {
                triggerMotorSet = 0.80d;
            }
        }

        if (true == beltMotorOn) {
            beltMotorSet = 0.5d;
        }

        m_feeder.setTriggerMotor(triggerMotorSet);
        m_feeder.setBeltMotor(beltMotorSet);
        writeData(triggerMotorSet, beltMotorSet, rpmSet, m_launcher.getCurrentRpm(), launcherReady);
    }

    private void writeData(double triggerMotorSet, double beltMotorSet, double rpmSet, double currentRpm, boolean launcherReady) {
        String stringToWrite = String.format("Fire,%d, %.2f, %.2f, %.2f, %.2f, %b, %.2f, %.2f, %b\n",
                System.currentTimeMillis(), triggerMotorSet, beltMotorSet,
                rpmSet, currentRpm, launcherReady,
                m_targeting.calcDistance(), m_targeting.getAngleToTargetDegrees(), m_targeting.isTargeted());

        m_logger.writeToFile(stringToWrite);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
