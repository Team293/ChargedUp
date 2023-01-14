package frc.robot.classes.spikemotor;

import com.ctre.phoenix.motorcontrol.can.TalonFX.*;

class SpikeMotorTalonFX extends SpikeMotor {
    private TalonFX motor;

    @Override
    protected void initImpl(int deviceNumber) {
        motor = new TalonFX(deviceNumber);
    }

    @Override
    protected void setSpeedImpl(double speed) {
        motor.set(TalonFXControlMode.Velocity, speed * MAX_ENCODER_VELOCITY);
    }

    @Override
    protected double getSpeedImpl() {
        return motor.getSelectedSensorVelocity() * WHEEL_CIRCUMFERENCE_FEET * DECISEC_TO_SECONDS / (GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION);
    }

    @Override
    protected double getPositionImpl() {
        return motor.getSelectedSensorPosition(0) * WHEEL_CIRCUMFERENCE_FEET / (GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION);
    }
}
