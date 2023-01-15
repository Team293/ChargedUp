package frc.robot.classes.spikemotor;

import com.ctre.phoenix.motorcontrol.can.TalonFX.*;
import frc.robot.classes.SPIKE293Utils;
import static frc.robot.Constants.DrivetrainConstants.*;

class SpikeMotorTalonFX extends SpikeMotor {
    private TalonFX motor;
    private double wheelDiameter;

    public SpikeMotorTalonFX(double wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }

    @Override
    protected void initImpl(int deviceNumber) {
        motor = new TalonFX(deviceNumber);
    }

    @Override
    protected void setSpeedImpl(double speed) {
        motor.set(TalonFXControlMode.Velocity, SPIKE293Utils.feetPerSecToControllerVelocity(speed, wheelDiameter));
    }

    @Override
    protected double getSpeedImpl() {
        return SPIKE293Utils.controllerVelocityToFeetPerSec(motor.getSelectedSensorVelocity(), wheelDiameter);
    }

    @Override
    protected double getPositionImpl() {
        return SPIKE293Utils.controllerUnitsToFeet(motor.getSelectedSensorPosition(0), wheelDiameter);
    }
}
