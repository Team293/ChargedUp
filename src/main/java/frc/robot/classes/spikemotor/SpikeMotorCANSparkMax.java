package frc.robot.classes.spikemotor;

import com.revrobotics.*;
import frc.robot.classes.SPIKE293Utils;
import static frc.robot.Constants.DrivetrainConstants.*;

class SpikeMotorCANSparkMax extends SpikeMotor {
    private CANSparkMax motor;
    private CANPIDController pidController;
    private double wheelDiameter;
    private boolean isBrushed;

    public SpikeMotorCANSparkMax(double wheelDiameter, boolean isBrushed) {
        this.wheelDiameter = wheelDiameter;
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
        pidController.setReference(SPIKE293Utils.convertFeetPerSecToRPM(speed, wheelDiameter), ControlType.kVelocity);
    }

    @Override
    protected double getSpeedImpl() {
        return SPIKE293Utils.convertRPMToFeetPerSec(motor.getEncoder().getVelocity(), wheelDiameter);
    }

    @Override
    protected double getPositionImpl() {
        return SPIKE293Utils.convertRotationsToFeet(motor.getEncoder().getPosition(), wheelDiameter);
    }
}
