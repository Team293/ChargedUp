package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Intake extends CommandBase {
    private static final int CAN_ID = 5;
    private WPI_TalonFX m_intakeMotor;
    private double m_leftCurrentSpeed;
    private double m_rightCurrentSpeed;
    private double m_targetRpms;
    private Drivetrain drivetrain;

    public Intake(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        m_intakeMotor = new WPI_TalonFX(CAN_ID);
        m_intakeMotor.clearStickyFaults();
        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.setInverted(false);
        m_intakeMotor.setSensorPhase(false);
        m_intakeMotor.config_kD(0, 0.021);
        m_intakeMotor.config_kI(0, 0.0001);
        m_intakeMotor.config_kP(0, 0.001);
        m_intakeMotor.config_kF(0, 0.068);
        Drivetrain.getTab().setBoolean("moving at same speed", false);
        Drivetrain.getTab().setDouble("left current speed", m_leftCurrentSpeed);
        Drivetrain.getTab().setDouble("right current speed", m_rightCurrentSpeed);
        Drivetrain.getTab().setDouble("total counteracting speed", m_targetRpms);
        m_intakeMotor.setNeutralMode(NeutralMode.Coast);
        m_intakeMotor.configNeutralDeadband(0);
    }

    @Override
    public void execute() {
        m_leftCurrentSpeed = drivetrain.getRawLeftEncoderVelocity();
        m_rightCurrentSpeed = drivetrain.getRawRightEncoderVelocity();
        Drivetrain.getTab().setDouble("left current speed", m_leftCurrentSpeed); 
        Drivetrain.getTab().setDouble("right current speed", m_rightCurrentSpeed);
        double totalSpeed = (m_leftCurrentSpeed  + m_rightCurrentSpeed) / 2;
        m_targetRpms = -200d - totalSpeed;
        MathUtil.clamp(totalSpeed, -10000000d, 0);
        Drivetrain.getTab().setDouble("total counteracting speed", m_targetRpms);
        m_intakeMotor.set(ControlMode.Velocity, -200d - totalSpeed);
        if (m_leftCurrentSpeed == m_rightCurrentSpeed) {
            Drivetrain.getTab().setBoolean("moving at same speed", true);
        } else {
            Drivetrain.getTab().setBoolean("moving at same speed", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeMotor.set(0.0d);
    }

    private double encoderToRpm(double encoderUnits) {
        return encoderUnits*600/2048d;
    }

    private double RpmToEncoder(double rpm) {
        return rpm*2048d/600d;
    }
}
