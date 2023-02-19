package frc.robot.classes.spikemotor;

import com.revrobotics.*;
import static frc.robot.Constants.TimeConstants.*;

public class SpikeMotorCANSparkMax extends SpikeMotor {
    public static final double VELOCITY_KF = 0.046d;
    public static final double VELOCITY_KP = 0.03d;
    public static final double VELOCITY_KI = 0.0d;
    public static final double VELOCITY_KD = 0.06d;

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
                isBrushed ? CANSparkMaxLowLevel.MotorType.kBrushed : CANSparkMaxLowLevel.MotorType.kBrushless);
        pidController = motor.getPIDController();
        pidController.setFF(VELOCITY_KF);
        pidController.setI(VELOCITY_KI);
        pidController.setD(VELOCITY_KD);
        pidController.setP(VELOCITY_KP);
    }

    @Override
    protected void setSpeedImpl(double speed) {
        pidController.setReference(speed * SECOND_TO_MINUTES / (Math.PI * conversionFactor),
                CANSparkMax.ControlType.kVelocity);
    }

    @Override
    protected double getSpeedImpl() {
        return (motor.getEncoder().getVelocity() * conversionFactor * MINUTE_TO_SECONDS);
    }

    @Override
    protected void setPositionImpl(double position) {
        motor.getEncoder().setPosition(position * SECOND_TO_MINUTES / (Math.PI * conversionFactor));
    }

    @Override
    protected double getPositionImpl() {
        return motor.getEncoder().getPosition() * MINUTE_TO_SECONDS * Math.PI * conversionFactor;
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
