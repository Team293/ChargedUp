package frc.robot.classes.spikemotor;

import edu.wpi.first.wpilibj.Servo;

public class SpikeMotorServo extends SpikeMotor {
    private Servo motor;
    private double offset = 0;
    private double wheelDiameter;
    
    public SpikeMotorServo(double wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }

    @Override
    protected void initImpl(int deviceNumber) {
        motor = new Servo(deviceNumber);
    }

    @Override
    protected void setSpeedImpl(double speed) {
        throw new UnsupportedOperationException("Cannot set the speed of a servo motor");
    }

    @Override
    protected double getSpeedImpl() {
        throw new UnsupportedOperationException("Cannot get the speed of a servo motor");
    }

    @Override
    protected void setPositionImpl(double position) {
        offset = position - servoUnitsToFeet(motor.getPosition());
    }

    @Override
    protected double getPositionImpl() {
        return servoUnitsToFeet(motor.getPosition()) + offset;
    }

    @Override
    protected void moveToImpl(double position) {
        motor.setPosition(feetToServoUnits(position));
    }

    private double servoUnitsToFeet(double servoUnits) {
        return servoUnits * Math.PI * wheelDiameter / 2;
    }

    private double feetToServoUnits(double feet) {
        return feet / Math.PI / wheelDiameter * 2;
    }

    @Override
    protected double getConversionFactor() {
        return wheelDiameter;
    }
}
