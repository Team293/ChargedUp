package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProximitySensor extends SubsystemBase {
        // Convert speed of sound to inches
    private double SPEED_OF_SOUND = 13503.9371d;
    public static final int ANALOG_CHANNEL = 0;
    private static final int TRIGGER_PIN = 0;

    private AnalogInput m_inputSensor;
    private DigitalOutput m_triggerPin;
    private double m_lastDistance = 0.0d;

    /**
     * Creates a new instance of the proximity sensor class.
     */
    public ProximitySensor() {
        m_inputSensor = new AnalogInput(ANALOG_CHANNEL);
        m_triggerPin = new DigitalOutput(TRIGGER_PIN);
        m_inputSensor.setAverageBits(4);
        SmartDashboard.putNumber("Ultrasonic - Speed of Sound (inches/sec)", SPEED_OF_SOUND);
    }

    /**
     * Gets the distance from an object that the sensor
     * @return approximate distance in inches.
     */
    public double getDistanceInches() {
        double time = getPingTime();
        SmartDashboard.putNumber("Ultrasonic - Response Time (sec)", time);
        SPEED_OF_SOUND = SmartDashboard.getNumber("Ultrasonic - Speed of Sound (inches/sec)", 13503.9371);

        double distance = (SPEED_OF_SOUND * time) / 2.0d;
        m_lastDistance = distance;
        return distance;
    }

    /**
     * Returns the estimated time taken by the sensor to send and recieve a ping.
     * @return time in seconds.
     */
    public double getPingTime() {
        sendPing();

        while (m_inputSensor.getVoltage() <= 0.5d) {
            // Wait until signal is high (sensor is sending a signal)
        }
        double startTime = Timer.getFPGATimestamp();
        while (m_inputSensor.getVoltage() > 0.5d) {
            // Wait until signal is low again
        }
        double endTime = Timer.getFPGATimestamp();

        double difference = endTime - startTime;
        return difference;
    }

    /**
     * Send a ping to the sensor.
     */
    public void sendPing() {
        // Send trigger pulse
        m_triggerPin.set(true);
        Timer.delay(0.00001);
        m_triggerPin.set(false);
    }

    /**
     * Get the last distance.
     */
    public double getLastDistance() {
        return m_lastDistance;
    }
}