package frc.robot.classes.spikemotor;

import com.revrobotics.*;
import static frc.robot.Constants.TimeConstants.*;
import java.util.HashMap;

public class SpikeMotorCANSparkMax extends SpikeMotor {
    public static final double VELOCITY_KF = 0.046d;
    public static final double VELOCITY_KP = 0.03d;
    public static final double VELOCITY_KI = 0.0d;
    public static final double VELOCITY_KD = 0.06d;

    private double conversionFactor;
    private CANSparkMax motor;
    private SparkMaxPIDController pidController;
    private boolean isBrushed;
    private HashMap<Integer, double[]> savedPid;

    public SpikeMotorCANSparkMax(double conversionFactor, boolean isBrushed) {
        this.conversionFactor = conversionFactor;
        this.isBrushed = isBrushed;
        savedPid = new HashMap<>();
    }

    @Override
    protected void initImpl(int deviceNumber) {
        motor = new CANSparkMax(
                deviceNumber,
                isBrushed ? CANSparkMaxLowLevel.MotorType.kBrushed : CANSparkMaxLowLevel.MotorType.kBrushless);
        pidController = motor.getPIDController();
        // pidController.setFF(VELOCITY_KF);
        // pidController.setI(VELOCITY_KI);
        // pidController.setD(VELOCITY_KD);
        // pidController.setP(VELOCITY_KP);
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

    @Override
    protected void setPidImpl(int slotId, double kP, double kI, double kD, double kF, double izone) {
        savedPid.put(slotId, new double[] { kP, kI, kD, kF, izone });
    }

    @Override
    protected void selectPidImpl(int slotId) {
        double[] pid = savedPid.get(slotId);
        pidController.setP(pid[0]);
        pidController.setI(pid[1]);
        pidController.setD(pid[2]);
        pidController.setFF(pid[3]);
        pidController.setIZone(pid[4]);
    }

    @Override
    protected void setClosedLoopRampImpl(double ramp) {
        motor.setClosedLoopRampRate(ramp);
    }

    @Override
    protected void setOpenLoopRampImpl(double ramp) {
        motor.setOpenLoopRampRate(ramp);
    }
}
