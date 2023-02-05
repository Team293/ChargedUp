package frc.robot.classes.spikemotor;

import com.revrobotics.*;
import static frc.robot.Constants.DrivetrainConstants.*;

public class SpikeMotorCANSparkMax extends SpikeMotor {
    private final double secondToMinute = 60.0d / 1.0d;
    private final double minuteToSecond = 1.0d / 60.0d;
    private double conversionFactor;
    private CANSparkMax motor;
    private SparkMaxPIDController pidController;
    private boolean isBrushed;

    public SpikeMotorCANSparkMax(double conversionFactor, boolean isBrushed) {
        this.conversionFactor = conversionFactor;
        this.isBrushed = isBrushed;
    }

    @Override
    protected void initImpl(int deviceNumber) {
        motor = new CANSparkMax(
            deviceNumber,
            isBrushed ?
                CANSparkMaxLowLevel.MotorType.kBrushed :
                CANSparkMaxLowLevel.MotorType.kBrushless
        );
        pidController = motor.getPIDController();
        pidController.setFF(VELOCITY_KF);
        pidController.setI(VELOCITY_KI);
        pidController.setD(VELOCITY_KD);
        pidController.setP(VELOCITY_KP);
    }

    @Override
    protected void setSpeedImpl(double speed) {
        pidController.setReference(speed * secondToMinute / (Math.PI * conversionFactor), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    protected double getSpeedImpl() {
        return (motor.getEncoder().getVelocity() * conversionFactor *  minuteToSecond);
    }

    @Override
    protected void setPositionImpl(double position) {
        motor.getEncoder().setPosition(position * secondToMinute / (Math.PI * conversionFactor));
    }

    @Override
    protected double getPositionImpl() {
        return motor.getEncoder().getPosition() / secondToMinute * Math.PI * conversionFactor;
    }

    @Override
    protected void moveToImpl(double position) {
        pidController.setReference(position / (Math.PI * conversionFactor), CANSparkMax.ControlType.kPosition);
    }

    @Override
    protected double getConversionFactor() {
        return conversionFactor;
    }
}
