package frc.robot.classes.spikemotor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import static frc.robot.Constants.TimeConstants.*;

import java.util.Optional;

public class SpikeMotorTalonFX extends SpikeMotor {
    public static final int CONFIG_FEEDBACKSENSOR_TIMEOUT_MS = 4000;
    public static final int PID_CONFIG_TIMEOUT_MS = 10;
    public static final double CLOSED_LOOP_RAMP = 0.5;
    public static final double MOTOR_NEUTRAL_DEADBAND = 0.001d;

    public static final double VELOCITY_KF = 0.046d;
    public static final double VELOCITY_KP = 0.03d;
    public static final double VELOCITY_KI = 0.0d;
    public static final double VELOCITY_KD = 0.06d;

    public static final double POSITION_KF = 0.0d;
    public static final double POSITION_KP = 0.029d;
    public static final double POSITION_KI = 0.0004d;
    public static final double POSITION_KD = 0.29d;

    public static final int VELOCITY_PID_SLOT_ID = 0;
    public static final int POSITION_PID_SLOT_ID = 1;

    private final double encoderUnitsPerRevolution = 2048.0d;
    private final double encoderUnitsToDecisec = encoderUnitsPerRevolution * SECOND_TO_DECISECS;
    private WPI_TalonFX motor;
    private double conversionFactor;
    private Boolean inverted;
    private InvertType invertType;

    public SpikeMotorTalonFX(double conversionFactor, boolean inverted) {
        this.conversionFactor = conversionFactor;
        this.inverted = inverted;
    }

    public SpikeMotorTalonFX(double conversionFactor, InvertType inverted) {
        this.conversionFactor = conversionFactor;
        this.invertType = inverted;
    }

    @Override
    protected void initImpl(int deviceNumber) {
        motor = new WPI_TalonFX(deviceNumber);
        motor.clearStickyFaults();
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_FEEDBACKSENSOR_TIMEOUT_MS);
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1,
                CONFIG_FEEDBACKSENSOR_TIMEOUT_MS);
        if (inverted != null) {
            motor.setInverted(inverted);
            motor.setSensorPhase(inverted);
        } else {
            motor.setInverted(invertType);
        }
        motor.setNeutralMode(NeutralMode.Coast);
        if (invertType != InvertType.FollowMaster) {
            motor.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        }
    }

    protected void addFollower(SpikeMotorTalonFX lead) {
        motor.follow(lead.motor);
    }

    @Override
    protected void setSpeedImpl(double speed) {
        motor.set(TalonFXControlMode.Velocity, conversionFactor * encoderUnitsToDecisec * speed);
    }

    @Override
    protected double getSpeedImpl() {
        return motor.getSelectedSensorVelocity() / conversionFactor / encoderUnitsToDecisec;
    }

    @Override
    protected void setPositionImpl(double position) {
        motor.setSelectedSensorPosition(conversionFactor * encoderUnitsPerRevolution * position);
    }

    @Override
    protected double getPositionImpl() {
        return motor.getSelectedSensorPosition(0) / conversionFactor / encoderUnitsPerRevolution;
    }

    @Override
    protected void moveToImpl(double position) {
        motor.set(ControlMode.Position, conversionFactor * encoderUnitsPerRevolution * position);
    }

    @Override
    protected double getConversionFactor() {
        return conversionFactor;
    }

    @Override
    protected void setPidImpl(int slotId, double kP, double kI, double kD, double kF, Optional<Double> izone) {
        motor.config_kF(slotId, kF, PID_CONFIG_TIMEOUT_MS);
        motor.config_kP(slotId, kP, PID_CONFIG_TIMEOUT_MS);
        motor.config_kI(slotId, kI, PID_CONFIG_TIMEOUT_MS);
        motor.config_kD(slotId, kD, PID_CONFIG_TIMEOUT_MS);
        if (izone.isPresent())
            motor.config_IntegralZone(slotId, izone.get(), PID_CONFIG_TIMEOUT_MS);
    }

    @Override
    protected void selectPidImpl(int slotId) {
        motor.selectProfileSlot(slotId, 0);
    }

    @Override
    protected void setClosedLoopRampImpl(double ramp) {
        motor.configClosedloopRamp(ramp);
    }

    @Override
    protected void setOpenLoopRampImpl(double ramp) {
        motor.configOpenloopRamp(ramp);
    }
}