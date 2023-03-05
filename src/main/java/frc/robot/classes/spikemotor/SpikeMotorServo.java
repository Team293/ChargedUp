package frc.robot.classes.spikemotor;

import java.util.Optional;

import edu.wpi.first.wpilibj.Servo;

public class SpikeMotorServo extends SpikeMotor {
    private Servo motor;
    private double offset = 0;
    private double conversionFactor;

    public SpikeMotorServo(double conversionFactor) {
        this.conversionFactor = conversionFactor;
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
        offset = position - servoUnitsToEncoder(motor.getPosition());
    }

    @Override
    protected double getPositionImpl() {
        return servoUnitsToEncoder(motor.getPosition()) + offset;
    }

    @Override
    protected void moveToImpl(double position) {
        motor.setPosition(encoderToServoUnits(position));
    }

    private double servoUnitsToEncoder(double servoUnits) {
        return servoUnits * Math.PI * conversionFactor / 2;
    }

    private double encoderToServoUnits(double feet) {
        return feet / Math.PI / conversionFactor * 2;
    }

    @Override
    protected double getConversionFactor() {
        return conversionFactor;
    }

    @Override
    protected void setPidImpl(int slotId, double kP, double kI, double kD, double kF, Optional<Double> izone) {
        // TODO
    }

    @Override
    protected void selectPidImpl(int slotId) {
        // TODO
    }

    @Override
    protected void setClosedLoopRampImpl(double ramp) {
        // TODO
    }

    @Override
    protected void setOpenLoopRampImpl(double ramp) {
        // TODO
    }
}