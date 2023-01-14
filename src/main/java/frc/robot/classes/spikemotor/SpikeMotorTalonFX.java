package frc.robot.classes.spikemotor;

import com.ctre.phoenix.motorcontrol.can.TalonFX.*;

class SpikeMotorTalonFX extends SpikeMotor {
    private TalonFX motor;

    @Override
    public void initImpl(int deviceNumber) {
        motor = new TalonFX(deviceNumber);
    }

    @Override
    public void setSpeedImpl(double speed) {
        motor.set(TalonFXControlMode.PercentOutput, speed);
    }

    @Override
    public double getSpeedImpl() {
        return motor.getMotorOutputPercent();
    }

    @Override
    public double getPositionImpl() {
        return motor.getSelectedSensorPosition(0);
    }
}
