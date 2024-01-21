package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Intake extends CommandBase {
    private static final int CAN_ID = 5;
    private WPI_TalonFX coleFx;
    private static final double MOTOR_SPEED = 0.1d;
    private double m_motorSpeed = 0.1d;

    public Intake() {
        coleFx = new WPI_TalonFX(CAN_ID);
        coleFx.clearStickyFaults();
        coleFx.configFactoryDefault();
        coleFx.setInverted(false);
        coleFx.setSensorPhase(false);
        coleFx.config_kD(0, 0.021);
        coleFx.config_kI(0, 0.0001);
        coleFx.config_kP(0, 0.001);
        Drivetrain.getTab().setDouble("intake motor speed", 0.1d);
        coleFx.setNeutralMode(NeutralMode.Coast);
        coleFx.configNeutralDeadband(0);
    }

    @Override
    public void execute() {
        m_motorSpeed = Drivetrain.getTab().getDouble("intake motor speed", 0.1d);
        coleFx.set(m_motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        coleFx.set(0.0d);
    }
}
