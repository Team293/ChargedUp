package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shooter extends CommandBase {
    private static final int CAN_ID = 0; //todo
    private WPI_TalonFX shooterMotor;
    private static final double MOTOR_SPEED = 0.1;

    public Shooter() {
        shooterMotor = new WPI_TalonFX(CAN_ID);
        shooterMotor.clearStickyFaults();
        shooterMotor.configFactoryDefault();
        shooterMotor.setInverted(false);
        shooterMotor.setSensorPhase(false);
        shooterMotor.config_kD(0, 0.021);
        shooterMotor.config_kI(0, 0.0001);
        shooterMotor.config_kP(0, 0.001);

        shooterMotor.setNeutralMode(NeutralMode.Coast);
        shooterMotor.configNeutralDeadband(0);
    }

    @Override
    public void execute() {
        shooterMotor.set(MOTOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooterMotor.set(0);
    }
}
