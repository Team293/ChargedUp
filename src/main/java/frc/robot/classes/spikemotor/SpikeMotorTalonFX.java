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
        motor.set(TalonFXControlMode.PercentOutput, speed);
    }

    @Override
    protected double getSpeedImpl() {
        return motor.getMotorOutputPercent();
    }

    @Override
    protected double getPositionImpl() {
        return motor.getSelectedSensorPosition(0);
    }
}
