package frc.robot.classes.spikemotor;

import com.revrobotics.*;

class SpikeMotorCANSparkMax extends SpikeMotor {
    private CANSparkMax motor;
    private boolean isBrushed;

    public SpikeMotorCANSparkMax(boolean isBrushed) {
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
    }

    @Override
    protected void setSpeedImpl(double speed) {
        motor.set(speed);
    }

    @Override
    protected double getSpeedImpl() {
        return motor.get(); // TODO: Update to get units in ft/sec
    }

    @Override
    protected double getPositionImpl() {
        return motor.getEncoder().getPosition(); // TODO: Update to get units in feet
    }
}
