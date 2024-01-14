package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.SpikeBoard;

public class Arm extends SubsystemBase {
    /* Constants */
    /* CAN IDs */
    public static final int PIVOT_TALON_FX_CAN_ID = 4;
    public static final int EXTENDER_TALON_FX_CAN_ID = 5;

    /* PIDs */
    public static final double PIVOT_KF = 0.7d;
    public static final double PIVOT_KP = 0.3d;
    public static final double PIVOT_KI = 0.001d;
    public static final double PIVOT_KD = 3d;

    public static final double EXTENDER_KF = 0.08d;
    public static final double EXTENDER_KP = 0.1d;
    public static final double EXTENDER_KI = 0.0025d;
    public static final double EXTENDER_KD = 1d;

    /* Velocity */
    public static final double PIVOT_MAX_VELOCITY = 5000.0d; // Controls the speed of the pivot
    public static final double PIVOT_MAX_ACCELERATION = 5000.0d; // Controls the acceleration of the pivot
    public static final double EXTENDER_MAX_VELOCITY = 10000.0d;
    public static final double EXTENDER_MAX_ACCELERATION = 35000.0d;

    /* Motor constants */
    public static final double PIVOT_MOTOR_NEUTRAL_DEADBAND = 0.001d;
    public static final double EXTENDER_MOTOR_NEUTRAL_DEADBAND = 0.0d;
    public static final int PID_CONFIG_TIMEOUT_MS = 10;
    public static final int CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS = 10;
    public static final double PIVOT_CALIBRATION_MOTOR_SPEED = 0.1d;
    public static final double EXTENDER_CALIBRATION_MOTOR_SPEED = 0.1d;

    /* Conversion Factors */
    public static final double ENCODER_UNITS_PER_REVOLUTION = 2048.0d / 1.0d;
    public static final double PIVOT_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO = 36.00d / 1.0d;
    public static final double PIVOT_PULLEY_MOTOR_TO_PULLEY_ARM_RATIO = 72.0d / 36.0d;
    public static final double EXTENDER_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO = 4.0d / 1.0d;
    public static final double EXTENDER_PULLEY_ROTATION_TO_INCHES = 3.75d; // One rotation of the final extender pulley
                                                                           // moves
    public static final double RADIANS_PER_REVOLUTION = 2 * Math.PI;
    public static final double PIVOT_ENCODER_UNITS_PER_RADIANS = (((ENCODER_UNITS_PER_REVOLUTION)
            * (PIVOT_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO) * (PIVOT_PULLEY_MOTOR_TO_PULLEY_ARM_RATIO))
            / (RADIANS_PER_REVOLUTION));
    public static final double EXTENDER_ENCODER_UNITS_PER_INCH = ((ENCODER_UNITS_PER_REVOLUTION
            * EXTENDER_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO) / EXTENDER_PULLEY_ROTATION_TO_INCHES);

    /* Physical constants */
    public static final double MIN_ANGLE_RADIANS = -90.0d * ((2 * Math.PI) / 360.0d); // radians
    public static final double MAX_ANGLE_RADIANS = 20.0d * ((2 * Math.PI) / 360.0d); // radians
    public static final double MIN_INCHES = 35.112d;
    public static final double MAX_INCHES = 50.0d;

    public static final double ARM_THETA_DELTA_MODIFIER = 1.0d * ((2 * Math.PI) / 360.0d); // radians
    public static final double ARM_R_DELTA_MODIFIER = 0.75d; // inches

    public static final double ZEROED_R_POSITION_RADIANS = 0.0d;
    public static final double ZEROED_THETA_POSITION_INCHES = -34.712d;

    public static final int ZEROED_PIVOT_ENCODER_LIMIT = (int) (MIN_ANGLE_RADIANS * PIVOT_ENCODER_UNITS_PER_RADIANS);
    public static final int ZEROED_EXTENDER_ENCODER_LIMIT = (int) (MIN_INCHES * EXTENDER_ENCODER_UNITS_PER_INCH);

    public static final double MIN_RESTRICTED_THETA = -72.0d * ((2 * Math.PI) / 360.0d); // radians
    public static final double MAX_RESTRICTED_INCHES = MIN_INCHES + 2.0d;

    public static final double STOW_ANGLE = MIN_ANGLE_RADIANS;
    public static final double STOW_R_INCHES = MIN_INCHES;

    /* Members */
    private WPI_TalonFX pivotTalonFX;
    private WPI_TalonFX extenderTalonFX;
    private static SpikeBoard armTab;
    private double theta = MIN_ANGLE_RADIANS;
    private double rInches = MIN_INCHES;
    private boolean isPivotCalibrated = false;
    private boolean isExtenderCalibrated = false;
    private double m_pivotCommandedEncoderUnits;
    private double m_extensionCommandedEncoderUnits;

    // Gear ratios
    public Arm() {
        // // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        pivotTalonFX = new WPI_TalonFX(PIVOT_TALON_FX_CAN_ID);
        extenderTalonFX = new WPI_TalonFX(EXTENDER_TALON_FX_CAN_ID);

        // // Clears motor errors
        // pivotTalonFX.clearStickyFaults();
        // extenderTalonFX.clearStickyFaults();

        // // Set factory defaults for onboard PID
        // pivotTalonFX.configFactoryDefault();
        // extenderTalonFX.configFactoryDefault();

        // pivotTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        //         CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);
        // extenderTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        //         CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);

        // pivotTalonFX.setInverted(true);
        // pivotTalonFX.setSensorPhase(true);
        // extenderTalonFX.setInverted(false);
        // extenderTalonFX.setSensorPhase(false);

        // // Configure Position PID
        // pivotTalonFX.config_kF(0, PIVOT_KF, PID_CONFIG_TIMEOUT_MS);
        // pivotTalonFX.config_kP(0, PIVOT_KP, PID_CONFIG_TIMEOUT_MS);
        // pivotTalonFX.config_kI(0, PIVOT_KI, PID_CONFIG_TIMEOUT_MS);
        // pivotTalonFX.config_kD(0, PIVOT_KD, PID_CONFIG_TIMEOUT_MS);
        // pivotTalonFX.config_IntegralZone(0, 1000);
        // pivotTalonFX.configMotionCruiseVelocity(PIVOT_MAX_VELOCITY, PID_CONFIG_TIMEOUT_MS);
        // pivotTalonFX.configMotionAcceleration(PIVOT_MAX_ACCELERATION, PID_CONFIG_TIMEOUT_MS);

        // extenderTalonFX.config_kF(0, EXTENDER_KF, PID_CONFIG_TIMEOUT_MS);
        // extenderTalonFX.config_kP(0, EXTENDER_KP, PID_CONFIG_TIMEOUT_MS);
        // extenderTalonFX.config_kI(0, EXTENDER_KI, PID_CONFIG_TIMEOUT_MS);
        // extenderTalonFX.config_kD(0, EXTENDER_KD, PID_CONFIG_TIMEOUT_MS);
        // extenderTalonFX.config_IntegralZone(0, 100);
        // extenderTalonFX.configMotionCruiseVelocity(EXTENDER_MAX_VELOCITY, PID_CONFIG_TIMEOUT_MS);
        // extenderTalonFX.configMotionAcceleration(EXTENDER_MAX_ACCELERATION, PID_CONFIG_TIMEOUT_MS);

        // extenderTalonFX.setNeutralMode(NeutralMode.Brake);
        // pivotTalonFX.setNeutralMode(NeutralMode.Brake);

        // extenderTalonFX.configNeutralDeadband(0);
        // pivotTalonFX.configNeutralDeadband(0);

        // // Start the calibration process
        // isPivotCalibrated = false;
        // isExtenderCalibrated = false;
    }

    public static SpikeBoard getTab() {
        if (armTab == null) {
            armTab = new SpikeBoard("Arm");
        }
        return armTab;
    }

    @Override
    public void periodic() {
        /* Update smart dashboard */
        Arm.getTab().setDouble("theta", theta);
        Arm.getTab().setDouble("R(inches)", rInches);
        Arm.getTab().setDouble("pivotMotor encoder", pivotTalonFX.getSelectedSensorPosition());
        Arm.getTab().setDouble("extender motor position", extenderTalonFX.getSelectedSensorPosition());
        Arm.getTab().setBoolean("Pivot Rev Limit", (1 == pivotTalonFX.isRevLimitSwitchClosed()));
        Arm.getTab().setBoolean("Extender Rev Limit", (1 == extenderTalonFX.isRevLimitSwitchClosed()));

        // Check both extender and pivot calibrations
        checkExtenderCalibration();
        checkPivotCalibration();

        /* Are we calibrated? */
        /*if (false == isExtenderCalibrated) {
            // The extender is not calibrated, start the extender calibration
            extenderTalonFX.set(-EXTENDER_CALIBRATION_MOTOR_SPEED);
        } else if (false == isPivotCalibrated) {
            // The extender is calibrated, but the pivot is not calibrated
            // Double Check: Is extender is pulled in?
            if (0 == extenderTalonFX.isRevLimitSwitchClosed()) {
                // Extender is not pulled in, pull it in
                extenderTalonFX.set(-EXTENDER_CALIBRATION_MOTOR_SPEED);
            } else {
                pivotTalonFX.set(-PIVOT_CALIBRATION_MOTOR_SPEED);
            }
        } else {
            // We are now calibrated and we know the position of the arm!
            moveToPosition(theta, rInches);
        }*/
    }

    /**
     * Rotate the arm to a specific angle.
     * 
     * @param angle - the angle in radians
     */
    public void rotateTo(double radians) {
        double minClamp = MIN_ANGLE_RADIANS;
        double encoderUnits = 0.0d;

        if (rInches >= (MAX_RESTRICTED_INCHES)) {
            minClamp = MIN_RESTRICTED_THETA;
        }

        /* Clamp the value to the max or min if needed */
        radians = Math.max(Math.min(radians, MAX_ANGLE_RADIANS), minClamp);

        /* Convert radians to encoder units */
        encoderUnits = radians * PIVOT_ENCODER_UNITS_PER_RADIANS;
        m_pivotCommandedEncoderUnits = encoderUnits;

        pivotTalonFX.set(TalonFXControlMode.MotionMagic, encoderUnits, DemandType.ArbitraryFeedForward,
                PIVOT_KF * Math.abs((Math.cos(radians))));
    }

    /**
     * Get the current encoder units.
     * 
     * @return
     */
    public double getPivotEncoderUnits() {
        double encoderUnits = pivotTalonFX.getSelectedSensorPosition();
        return encoderUnits;
    }

    public double getExtenderEncoderUnits() {
        double encoderUnits = extenderTalonFX.getSelectedSensorPosition();
        return encoderUnits;
    }

    /**
     * Extends arm to a given amount of inches.
     * 
     * @param inches - the length to extend/retract to in inches
     */
    public void extendTo(double inches) {
        double maxClamp = MAX_INCHES;
        double encoderUnits;

        /* Prevent a collision with the robot bumpers */
        if (theta <= MIN_RESTRICTED_THETA) {
            maxClamp = MAX_RESTRICTED_INCHES;
        }

        // Convert from inches to encoder units
        encoderUnits = inches * EXTENDER_ENCODER_UNITS_PER_INCH;
        m_extensionCommandedEncoderUnits = encoderUnits;

        /* Clamp the value to the max or min if needed */
        inches = Math.max(Math.min(inches, maxClamp), MIN_INCHES);

        SmartDashboard.putNumber("extender encoder", encoderUnits);
        extenderTalonFX.set(TalonFXControlMode.MotionMagic, encoderUnits, DemandType.ArbitraryFeedForward,
                EXTENDER_KF * Math.abs((Math.sin(theta))));
    }

    /**
     * Returns the position that the arm wants to reach, in encoder units.s
     * 
     * @returns double
     */
    public double getCommandedPivotEncoderPosition() {
        return m_pivotCommandedEncoderUnits;
    }

    public double getCommandedExtenderEncoderPosition() {
        return m_extensionCommandedEncoderUnits;
    }

    /**
     * Moves the arm to a given position in space with the arm pivot as the origin.
     * 
     * @param targetTheta   - target position of arm in radians
     * @param targetRInches - target position of the extension in inches
     */
    public void moveToPosition(double targetTheta, double targetRInches) {
        rotateTo(targetTheta);
        extendTo(targetRInches);
    }

    /**
     * Sets the internal position of the arm.
     * 
     * @param targetTheta   - target angle of arm in radians
     * @param targetRInches - target position of the extension in inches
     */
    public void setPosition(double targetTheta, double targetRInches) {
        theta = targetTheta;
        rInches = targetRInches;
    }

    public void adjustPosition(double anglePercent, double extendPercent) {
        theta += ARM_THETA_DELTA_MODIFIER * anglePercent;
        rInches += ARM_R_DELTA_MODIFIER * extendPercent;
    }

    /**
     * Stows the arm within the robot
     */
    public void stowArm() {
        rInches = STOW_R_INCHES;
        theta = STOW_ANGLE;
    }

    public void pivotSetEncoderUnits(int encoderUnits) {
        pivotTalonFX.setSelectedSensorPosition(encoderUnits);
    }

    public void extenderSetEncoderUnits(int encoderUnits) {
        extenderTalonFX.setSelectedSensorPosition(encoderUnits);
    }

    public void invalidatePivotCalibration() {
        isPivotCalibrated = false;
    }

    public void invalidateExtenderCalibration() {
        isExtenderCalibrated = false;
    }

    /*
     * Checks if the Extender motor is calibrated '
     * Does nothing if isExtenderCalibrated is true
     * If false, once the reverse Extender limt switch is tripped the Extender motor
     * is stopped, and the position is set
     */
    private void checkExtenderCalibration() {
        // Do we need to check for calibration?
        if (false == isExtenderCalibrated) {
            // The extender is not calibrated! Check to see if it is now calibrated
            // Is limit switch closed?
            if (extenderTalonFX.isRevLimitSwitchClosed() == 1) {
                // Stop moving motor
                extenderTalonFX.set(0);
                extenderSetEncoderUnits(ZEROED_EXTENDER_ENCODER_LIMIT);

                // We know where rInches is now, set it
                rInches = MIN_INCHES;

                // Extender is now calibrated!
                isExtenderCalibrated = true;
            }
        }
    }

    /*
     * Checks if the pivot motor is calibrated '
     * Does nothing if isPivotCalibrated is true
     * If false, once the reverse pivot limt switch is tripped the pivot motor is
     * stopped, and the position is set
     */
    private void checkPivotCalibration() {
        // Do we need to check for calibration?
        if (false == isPivotCalibrated) {
            // The pivot is not calibrated! Check to see if it is now calibrated
            // Is limit switch closed?
            if (pivotTalonFX.isRevLimitSwitchClosed() == 1) {
                // Stop moving motor
                pivotTalonFX.set(0);
                pivotSetEncoderUnits(ZEROED_PIVOT_ENCODER_LIMIT);

                // We know where theta is now, set it
                theta = MIN_ANGLE_RADIANS;

                // Pivot is now calibrated!
                isPivotCalibrated = true;
            }
        }
    }

    public boolean isCalibrated() {
        return ((true == isPivotCalibrated) && (true == isExtenderCalibrated));
    }

    public double getTheta() {
        return theta;
    }

    public double getRInches() {
        return rInches;
    }
}