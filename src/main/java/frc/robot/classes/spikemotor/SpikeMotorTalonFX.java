package frc.robot.classes.spikemotor;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import static frc.robot.Constants.DrivetrainConstants.*;

public class SpikeMotorTalonFX extends SpikeMotor {
    private final double encoderUnitsPerRevolution = 2048.0d;
    private final double encoderUnitsToDecisec = encoderUnitsPerRevolution * SECONDS_TO_DECISEC;
    private TalonFX motor;
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
        motor = new TalonFX(deviceNumber);
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
            motor.config_kF(VELOCITY_PID_SLOT_ID, VELOCITY_KF, PID_CONFIG_TIMEOUT_MS);
            motor.config_kP(VELOCITY_PID_SLOT_ID, VELOCITY_KP, PID_CONFIG_TIMEOUT_MS);
            motor.config_kI(VELOCITY_PID_SLOT_ID, VELOCITY_KI, PID_CONFIG_TIMEOUT_MS);
            motor.config_kD(VELOCITY_PID_SLOT_ID, VELOCITY_KD, PID_CONFIG_TIMEOUT_MS);
            motor.configClosedloopRamp(CLOSED_LOOP_RAMP);
            motor.configOpenloopRamp(0.2);
            motor.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
            motor.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
            motor.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
            motor.config_IntegralZone(POSITION_PID_SLOT_ID, 1000);
            motor.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);
            motor.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        }
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
}
