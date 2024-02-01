package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Shooter extends CommandBase {
    private static final int CAN_ID = 5;
    private WPI_TalonFX m_shooterMotor;
    private double m_targetRpms = 0.0d;
    private double m_currentRpms = 0.0d;

    public Shooter() {
        m_shooterMotor = new WPI_TalonFX(CAN_ID);
        m_shooterMotor.clearStickyFaults();
        m_shooterMotor.configFactoryDefault();
        m_shooterMotor.setInverted(false);
        m_shooterMotor.setSensorPhase(false);
        m_shooterMotor.config_kD(0, 0.021);
        m_shooterMotor.config_kI(0, 0.0001);
        m_shooterMotor.config_kP(0, 0.001);
        m_shooterMotor.config_kF(0, 0.068);
        Drivetrain.getTab().setDouble("shooter target rpm", m_targetRpms);
        Drivetrain.getTab().setDouble("shooter current rpm", m_currentRpms);

        m_shooterMotor.setNeutralMode(NeutralMode.Coast);
        m_shooterMotor.configNeutralDeadband(0);
    }

    @Override
    public void execute() {
        m_targetRpms = Drivetrain.getTab().getDouble("shooter target rpm", 0.1d);
        double encoderSpeed = RpmToEncoder(m_targetRpms);
        m_shooterMotor.set(ControlMode.Velocity, encoderSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterMotor.set(0.0d);
    }

    private double encoderToRpm(double encoderUnits) {
        return encoderUnits*600/2048d;
    }

    private double RpmToEncoder(double rpm) {
        return rpm*2048d/600d;
    }
}
