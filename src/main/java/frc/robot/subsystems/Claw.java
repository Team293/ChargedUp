package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    /* Constants */
    public final int PID_CONFIG_TIMEOUT_MS = 10;
    public final int CLAW_TALON_FX_CAN_ID = 6;
    public final int CONFIG_CLAW_FEEDBACKSENSOR_TIMEOUT_MS = 4000;
    private WPI_TalonFX clawTalonFX;

    private final double CLAW_LIMIT_PERCENTAGE = 0.4;

    public Claw() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        clawTalonFX = new WPI_TalonFX(CLAW_TALON_FX_CAN_ID);

        // Clears motor errors
        clawTalonFX.clearStickyFaults();

        clawTalonFX.setNeutralMode(NeutralMode.Brake);

        // Set factory defaults for onboard PID
        clawTalonFX.configFactoryDefault();

        clawTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_CLAW_FEEDBACKSENSOR_TIMEOUT_MS);

        clawTalonFX.setInverted(false);
        clawTalonFX.setSensorPhase(false);

        SmartDashboard.putNumber("Claw Percent Output", CLAW_LIMIT_PERCENTAGE);
    }

    @Override
    public void periodic() {
    }

    public void percentClaw(double percentage) {
        clawTalonFX.set(ControlMode.PercentOutput, percentage * CLAW_LIMIT_PERCENTAGE);
    }
}