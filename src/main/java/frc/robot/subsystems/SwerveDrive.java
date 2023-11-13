package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.Position2D;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.classes.SpikeBoard;

import java.util.Collections;
import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.Logger;

import frc.robot.classes.Kinematics;

public class SwerveDrive extends SubsystemBase {
    private WPI_TalonFX leftTalonLeadTurn;
    private WPI_TalonFX leftTalonLeadDrive;
    private WPI_TalonFX rightTalonLeadTurn;
    private WPI_TalonFX rightTalonLeadDrive;
    private WPI_TalonFX leftTalonFollowerTurn;
    private WPI_TalonFX leftTalonFollowerDrive;
    private WPI_TalonFX rightTalonFollowerTurn;
    private WPI_TalonFX rightTalonFollowerDrive;

    private Kinematics kinematics;
    private AHRS navX;
    private static SpikeBoard driveTab;

    public static final int LEFT_TALON_LEAD_TURN_CAN_ID = 0;
    public static final int LEFT_TALON_LEAD_DRIVE_CAN_ID = 1;
    public static final int RIGHT_TALON_LEAD_TURN_CAN_ID = 2;
    public static final int RIGHT_TALON_LEAD_DRIVE_CAN_ID = 3;
    public static final int LEFT_TALON_FOLLOWER_TURN_CAN_ID = 4;
    public static final int LEFT_TALON_FOLLOWER_DRIVE_CAN_ID = 5;
    public static final int RIGHT_TALON_FOLLOWER_TURN_CAN_ID = 6;
    public static final int RIGHT_TALON_FOLLOWER_DRIVE_CAN_ID = 7;

    public static final double VELOCITY_KF = 0.046d;
    public static final double VELOCITY_KP = 0.03d;
    public static final double VELOCITY_KI = 0.0d;
    public static final double VELOCITY_KD = 0.06d;
    
    public static final double POSITION_KF = 0.0d;
    public static final double POSITION_KP = 0.028d;
    public static final double POSITION_KI = 0.0004d;
    public static final double POSITION_KD = 0.29d;

    public static final int VELOCITY_PID_SLOT_ID = 0;
    public static final int POSITION_PID_SLOT_ID = 1;
    
    public static final int PID_CONFIG_TIMEOUT_MS = 10;

    public static final double MOTOR_NEUTRAL_DEADBAND = 0.001d;

    public static final boolean USE_NAVX_HEADING = false;

    public static final double getGyroHeadingDegrees  = 0.0d;

    public static final double TRACK_WIDTH_FEET = 27.5d / 12.0d;

    public SwerveDrive(Kinematics kinematics) {
        leftTalonLeadTurn = new WPI_TalonFX(LEFT_TALON_LEAD_TURN_CAN_ID);
        leftTalonLeadDrive = new WPI_TalonFX(LEFT_TALON_LEAD_DRIVE_CAN_ID);
        rightTalonLeadTurn = new WPI_TalonFX(RIGHT_TALON_LEAD_TURN_CAN_ID);
        rightTalonLeadDrive = new WPI_TalonFX(RIGHT_TALON_LEAD_DRIVE_CAN_ID);
        leftTalonFollowerTurn = new WPI_TalonFX(LEFT_TALON_FOLLOWER_TURN_CAN_ID);
        leftTalonFollowerDrive = new WPI_TalonFX(LEFT_TALON_FOLLOWER_DRIVE_CAN_ID);
        rightTalonFollowerTurn = new WPI_TalonFX(RIGHT_TALON_FOLLOWER_TURN_CAN_ID);
        rightTalonFollowerDrive = new WPI_TalonFX(RIGHT_TALON_FOLLOWER_DRIVE_CAN_ID);
        leftTalonLeadTurn.clearStickyFaults();
        leftTalonLeadDrive.clearStickyFaults();

        // Set factory defaults for onboard PID
        leftTalonLeadTurn.configFactoryDefault();
        leftTalonLeadDrive.configFactoryDefault();

        for (WPI_TalonFX motor : new WPI_TalonFX[] {leftTalonLeadTurn, leftTalonLeadDrive}) {
            motor.config_kF(VELOCITY_PID_SLOT_ID, VELOCITY_KF, PID_CONFIG_TIMEOUT_MS);
            motor.config_kP(VELOCITY_PID_SLOT_ID, VELOCITY_KP, PID_CONFIG_TIMEOUT_MS);
            motor.config_kI(VELOCITY_PID_SLOT_ID, VELOCITY_KI, PID_CONFIG_TIMEOUT_MS);
            motor.config_kD(VELOCITY_PID_SLOT_ID, VELOCITY_KD, PID_CONFIG_TIMEOUT_MS);
            motor.configOpenloopRamp(0.2);

            motor.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
            motor.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
            motor.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
            motor.config_IntegralZone(POSITION_PID_SLOT_ID, 1000);
            motor.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);
        }

        for (WPI_TalonFX motor : new WPI_TalonFX[] {
            leftTalonLeadTurn, leftTalonLeadDrive, rightTalonLeadTurn, rightTalonLeadDrive,
            leftTalonFollowerTurn, leftTalonFollowerDrive, rightTalonFollowerTurn, rightTalonFollowerDrive
        }) {
            motor.setNeutralMode(NeutralMode.Coast);
        }

        leftTalonLeadTurn.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        leftTalonLeadDrive.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);

        navX = new AHRS(SerialPort.Port.kMXP);

        this.kinematics = kinematics;

        // Since the turn motors should all be going the same direction:
        rightTalonLeadTurn.follow(leftTalonLeadTurn);
        leftTalonFollowerTurn.follow(leftTalonLeadTurn);
        rightTalonFollowerTurn.follow(leftTalonLeadTurn);
        
        // Since the drive motors should all drive simultaneously:
        rightTalonLeadDrive.follow(leftTalonLeadDrive);
        leftTalonFollowerDrive.follow(leftTalonLeadDrive);
        rightTalonFollowerDrive.follow(leftTalonLeadDrive);
    }
    
    public static SpikeBoard getTab() {
        if (driveTab == null) {
            driveTab = new SpikeBoard("Drivetrain");
        }
        return driveTab;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Put code here to be run every loop
        // Run Kinematics
        if (USE_NAVX_HEADING) {
            // use the NAVX for heading
            double headingInRadians = Math.toRadians(getGyroHeadingDegrees());
            kinematics.calculatePosition(getLeftTurnEncoderPosition(), getRightTurnEncoderPosition(), headingInRadians);
            kinematics.calculatePosition(getLeftDriveEncoderPosition(), getRightDriveEncoderPosition(), headingInRadians);

        } else {
            // Use the encoder information for heading
            kinematics.calculatePosition(getLeftTurnEncoderPosition(), getRightTurnEncoderPosition());
            kinematics.calculatePosition(getLeftDriveEncoderPosition(), getRightDriveEncoderPosition());
        }

        // Get current pose from Kinematics
        Position2D currentPose = kinematics.getPose();

        // Log Position w/ Pose2d class
        Pose2d currentPose2d = new Pose2d(currentPose.getX(), currentPose.getY(),
                Rotation2d.fromDegrees(currentPose.getHeadingDegrees()));
        Logger.getInstance().recordOutput("odometry", currentPose2d);

        Hashtable<String, Double> doubleVals = new Hashtable<String, Double>() {
            {
                put("Kinematics X (Feet)", currentPose.getX());
                put("Kinematics Y (Feet)", currentPose.getY());
                put("Kinematics Heading (degrees)", currentPose.getHeadingDegrees());
                put("Left Turn Encoder Velocity (Ft per S)", getLeftTurnEncoderVelocity());
                put("Left Turn Encoder Position (Ft)", getLeftTurnEncoderPosition());
                put("Left Drive Encoder Velocity (Ft per S)", getLeftDriveEncoderVelocity());
                put("Left Drive Encoder Position (Ft)", getLeftDriveEncoderPosition());
                put("Right Turn Encoder Veloctiy (Ft per S)", getRightTurnEncoderVelocity());
                put("Right Turn Encoder Position (Ft)", getRightTurnEncoderPosition());
                put("Right Drive Encoder Veloctiy (Ft per S)", getRightDriveEncoderVelocity());
                put("Right Drive Encoder Position (Ft)", getRightDriveEncoderPosition());

                put("Raw Left Turn Encoder", leftTalonLeadTurn.getSelectedSensorPosition(0));
                put("Raw Right Turn Encoder", rightTalonLeadTurn.getSelectedSensorPosition(0));
                put("Raw Left Drive Encoder", leftTalonLeadDrive.getSelectedSensorPosition(0));
                put("Raw Right Drive Encoder", rightTalonLeadDrive.getSelectedSensorPosition(0));

                put("Robot Heading (degrees)", getGyroHeadingDegrees());
                put("NavX X Accel", (double) navX.getWorldLinearAccelX());
                put("NavX Y Accel", (double) navX.getWorldLinearAccelY());
                put("NavX Z Accel", (double) navX.getWorldLinearAccelZ());
                put("NavX Yaw", getGyroYawDegrees());
                put("NavX Pitch", getGyroPitchDegrees());
                put("NavX Angle", getGyroHeadingDegrees());
                put("NavX Fused Heading", getGyroFusedHeadingDegrees());
                put("NavX TurnRate dg per s", navX.getRate());

                put("Left Motor Turn Position Error", leftTalonLeadTurn.getClosedLoopError(0));
                put("Right Motor Drive Position Error", rightTalonLeadDrive.getClosedLoopError(0));
                put("Left Motor Drive Position Error", leftTalonLeadDrive.getClosedLoopError(0));
                put("Right Motor Turn Position Error", rightTalonLeadTurn.getClosedLoopError(0));

                // Since the turn motors should all be going the same direction:
            rightTalonLeadTurn.follow(leftTalonLeadTurn);
            leftTalonFollowerTurn.follow(leftTalonLeadTurn);
            rightTalonFollowerTurn.follow(leftTalonLeadTurn);
            
            // Since the drive motors should all drive simultaneously:
            rightTalonLeadDrive.follow(leftTalonLeadDrive);
            leftTalonFollowerDrive.follow(leftTalonLeadDrive);
            rightTalonFollowerDrive.follow(leftTalonLeadDrive);
            }
            
        };
        
        Enumeration<String> doubleValsKeys = doubleVals.keys();
        for (String key : Collections.list(doubleValsKeys)) {
            double val = doubleVals.get(key);
            SmartDashboard.putNumber(key, val);
            Drivetrain.getTab().setDouble(key, val);
        }
    }

        /**
     * returns left encoder position
     * 
     * @return left encoder position
     */
    public double getLeftTurnEncoderPosition() {
        // Returns the number of steps, multiply by edges per step to get EPR, divided
        // by the gearbox ratio
        return SPIKE293Utils.controllerUnitsToFeet(leftTalonLeadTurn.getSelectedSensorPosition(0));
    }

    public double getLeftDriveEncoderPosition() {
        // Returns the number of steps, multiply by edges per step to get EPR, divided
        // by the gearbox ratio
        return SPIKE293Utils.controllerUnitsToFeet(leftTalonLeadDrive.getSelectedSensorPosition(0));
    }
    /**
     * returns right encoder position
     * 
     * @return right encoder position
     */
    public double getRightTurnEncoderPosition() {
        // Returns the number of steps, multiply by edges per step to get EPR, divided
        // by the gearbox ratio
        return SPIKE293Utils.controllerUnitsToFeet(rightTalonLeadTurn.getSelectedSensorPosition(0));
    }

    public double getRightDriveEncoderPosition() {
        // Returns the number of steps, multiply by edges per step to get EPR, divided
        // by the gearbox ratio
        return SPIKE293Utils.controllerUnitsToFeet(rightTalonLeadDrive.getSelectedSensorPosition(0));
    }


    /**
     * returns left encoder Velocity in ft/s
     * 
     * @return left encoder Velocity in ft/s
     */
    public double getLeftTurnEncoderVelocity() {
        // Returns the velocity of encoder by claculating the velocity from encoder
        // units of click/100ms to ft/s
        return SPIKE293Utils.controllerVelocityToFeetPerSec(leftTalonLeadTurn.getSelectedSensorVelocity());
    }

        public double getLeftDriveEncoderVelocity() {
        // Returns the velocity of encoder by claculating the velocity from encoder
        // units of click/100ms to ft/s
        return SPIKE293Utils.controllerVelocityToFeetPerSec(leftTalonLeadDrive.getSelectedSensorVelocity());
    }

    /**
     * returns right encoder Velocity in ft/s
     * 
     * @return right encoder Velocity in ft/s
     */
    public double getRightTurnEncoderVelocity() {
        // Returns the velocity of encoder by claculating the velocity from encoder
        // units of click/100ms to ft/s
        return SPIKE293Utils.controllerVelocityToFeetPerSec(rightTalonLeadTurn.getSelectedSensorVelocity()) ;
    }
    
    public double getRightDriveEncoderVelocity() {
        // Returns the velocity of encoder by claculating the velocity from encoder
        // units of click/100ms to ft/s
        return SPIKE293Utils.controllerVelocityToFeetPerSec(rightTalonLeadDrive.getSelectedSensorVelocity()) ;
    }


    /**
     * returns robot Velocity in ft/s
     * 
     * @return robot Velocity in ft/s
     */
    public double getRobotVelocity() {
        // Returns the velocity of the robot by taking the averga of the velcity on both
        // sides of the robor
        return (getLeftDriveEncoderVelocity() + getRightDriveEncoderVelocity()) / 2.0d;
    }

    /**
     * resets the drive train encoders to 0
     */
    private void zeroDriveTrainEncoders() {
        leftTalonLeadTurn.setSelectedSensorPosition(0);
        leftTalonLeadDrive.setSelectedSensorPosition(0);
        rightTalonLeadTurn.setSelectedSensorPosition(0);
        rightTalonLeadDrive.setSelectedSensorPosition(0);
    }

    public double getGyroFusedHeadingDegrees() {
        return (navX.getFusedHeading() * -1.0d);
    }

    public double getGyroPitchDegrees() {
        return (navX.getPitch() * -1.0d);
    }

    public double getGyroYawDegrees() {
        return (navX.getYaw() * -1.0d);
    }

    public double getGyroHeadingDegrees() {
        return (navX.getAngle() * -1.0d);
    }

    public void setupGyro(AHRS gyro, double startingAngleDegrees) {

        System.out.println("Calibrating gyroscope.");

        gyro.enableBoardlevelYawReset(true);
        gyro.reset();
        gyro.calibrate();

        // Wait for gyro to calibrate ~1 - 10 seconds
        while (gyro.isCalibrating()) {
        }

        // Set's the starting angle to the given angle
        gyro.setAngleAdjustment(startingAngleDegrees);

        System.out.println("Calibrating gyroscope done.");
    }

    public void resetKinematics() {
        setupGyro(navX, 0.0d);
        zeroDriveTrainEncoders();
    }

    public void resetGyro(double headingDegrees) {
        setupGyro(navX, headingDegrees);
    }

     // rotates robot according to give degress using arc length formula
    public void rotateDegrees(double angle) {
        double radians = Math.toRadians(angle);
        double arcLength = (radians * (TRACK_WIDTH_FEET / 2.0));
        double encoderTicks = SPIKE293Utils.feetToControllerUnits(arcLength);
        double leftEncoderTurnPosition = leftTalonLeadTurn.getSelectedSensorPosition(0);
        double rightEncoderTurnPosition = rightTalonLeadTurn.getSelectedSensorPosition(0);
        double leftEncoderDrivePosition = leftTalonLeadDrive.getSelectedSensorPosition(0);
        double rightEncoderDrivePosition = rightTalonLeadDrive.getSelectedSensorPosition(0);
        positionControl(leftEncoderTurnPosition - encoderTicks, rightEncoderTurnPosition + encoderTicks);
        positionControl(leftEncoderDrivePosition - encoderTicks, rightEncoderDrivePosition + encoderTicks);
    }

    // sets left and right talons to given parameters
    public void positionControl(double posL, double posR) {
        leftTalonLeadTurn.selectProfileSlot(POSITION_PID_SLOT_ID, 0);
        rightTalonLeadTurn.selectProfileSlot(POSITION_PID_SLOT_ID, 0);
        leftTalonLeadDrive.selectProfileSlot(POSITION_PID_SLOT_ID, 0);
        rightTalonLeadDrive.selectProfileSlot(POSITION_PID_SLOT_ID, 0);

        leftTalonLeadTurn.set(ControlMode.Position, posL);
        rightTalonLeadTurn.set(ControlMode.Position, posR);
        leftTalonLeadDrive.set(ControlMode.Position, posL);
        rightTalonLeadDrive.set(ControlMode.Position, posR);
    }

    public double getLeftTurnMotorError() {
        return leftTalonLeadTurn.getClosedLoopError(0);
    }

    public double getLeftDriveMotorError() {
        return leftTalonLeadDrive.getClosedLoopError(0);
    }

    public double getRightTurnMotorError() {
        return rightTalonLeadTurn.getClosedLoopError(0);
    }

    public double getRightDriveMotorError() {
        return rightTalonLeadDrive.getClosedLoopError(0);
    }
}
