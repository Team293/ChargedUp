package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.ArmConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private WPI_TalonFX shoulderTalonFX;
    private WPI_TalonFX reachTalonFX;
    private double x;
    private double y;

    public Arm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        shoulderTalonFX = new WPI_TalonFX(SHOULDER_TALON_FX_CAN_ID);
        reachTalonFX = new WPI_TalonFX(REACH_TALON_FX_CAN_ID);

        shoulderTalonFX.clearStickyFaults();
        reachTalonFX.clearStickyFaults();

        // Set facotry defaults for onboard PID
        shoulderTalonFX.configFactoryDefault();
        reachTalonFX.configFactoryDefault();

        shoulderTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);
        reachTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);

        shoulderTalonFX.setInverted(false);
        shoulderTalonFX.setSensorPhase(false);
        reachTalonFX.setInverted(true);
        reachTalonFX.setSensorPhase(true);

        // Configure Position PID
        shoulderTalonFX.config_kF(0, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
        shoulderTalonFX.config_kP(0, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
        shoulderTalonFX.config_kI(0, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
        shoulderTalonFX.config_IntegralZone(0, 1000);
        shoulderTalonFX.config_kD(0, POSITION_KD, PID_CONFIG_TIMEOUT_MS);

        reachTalonFX.config_kF(0, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
        reachTalonFX.config_kP(0, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
        reachTalonFX.config_kI(0, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
        reachTalonFX.config_IntegralZone(0, 1000);
        reachTalonFX.config_kD(0, POSITION_KD, PID_CONFIG_TIMEOUT_MS);

        reachTalonFX.setNeutralMode(NeutralMode.Brake);
        shoulderTalonFX.setNeutralMode(NeutralMode.Brake);

        reachTalonFX.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        shoulderTalonFX.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
    }

    @Override
    public void periodic() {
        moveToPosition(x,y);
    }

    /**
     * Rotate the arm to a specific angle.
     * @param angle - the angle in degrees
     */
    public void rotateTo(double angle) {

    }

    /**
     * Extends arm to a given amount of inches.
     * @param length - the length to extend/retract to in inches
     */
    public void extendTo(double length) {
        
    }

    /**
     * Moves the arm to a given position in space relative to the base.
     * 
     * @param targetX - position in inches
     * @param targetY - position in inches
     */
    public void moveToPosition(double targetX, double targetY) {
        double xDist = targetX - ARM_SHOULDER_X_INCHES;
        double yDist = targetY - ARM_SHOULDER_Y_INCHES;
        double length = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        double theta = Math.atan2(yDist, xDist);

        rotateTo(theta);
        extendTo(length);
    }

    /**
     * Sets the internal position of the arm.
     * @param xPos - position in inches
     * @param yPos - position in inches
     */
    public void setPosition(double xPos, double yPos) {
        x = xPos;
        y = yPos;
    }

    public void adjustPosition(double xPercentage, double yPercentage) {
        x += ARM_X_DELTA_MODIFIER * xPercentage;
        y += ARM_Y_DELTA_MODIFIER * yPercentage;
    }
}
