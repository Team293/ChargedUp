// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.SPIKE293Utils;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import static frc.robot.Constants.DrivetrainConstants.*;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX leftTalonLead;
    private WPI_TalonFX rightTalonLead;
    private WPI_TalonFX leftTalonFollower;
    private WPI_TalonFX rightTalonFollower;
    private AHRS navX;
    private Kinematics m_kinematics;

    public Drivetrain(Kinematics kinematics) {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        leftTalonLead = new WPI_TalonFX(LEFT_LEAD_TALON_CAN_ID);
        rightTalonLead = new WPI_TalonFX(RIGHT_LEAD_TALON_CAN_ID);
        leftTalonFollower = new WPI_TalonFX(LEFT_FOLLOWER_TALON_CAN_ID);
        rightTalonFollower = new WPI_TalonFX(RIGHT_FOLLOWER_TALON_CAN_ID);

        leftTalonLead.clearStickyFaults();
        rightTalonLead.clearStickyFaults();

        // Set facotry defaults for onboard PID
        leftTalonLead.configFactoryDefault();
        rightTalonLead.configFactoryDefault();

        leftTalonLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_FEEDBACKSENSOR_TIMEOUT_MS);
        rightTalonLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_FEEDBACKSENSOR_TIMEOUT_MS);

        leftTalonLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1,
                CONFIG_FEEDBACKSENSOR_TIMEOUT_MS);
        rightTalonLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1,
                CONFIG_FEEDBACKSENSOR_TIMEOUT_MS);

        leftTalonLead.setInverted(false);
        leftTalonLead.setSensorPhase(false);
        leftTalonFollower.setInverted(InvertType.FollowMaster);
        rightTalonLead.setInverted(true);
        rightTalonLead.setSensorPhase(true);
        rightTalonFollower.setInverted(InvertType.FollowMaster);

        // Configure Velocity PID
        leftTalonLead.config_kF(VELOCITY_PID_SLOT_ID, VELOCITY_KF, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.config_kP(VELOCITY_PID_SLOT_ID, VELOCITY_KP, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.config_kI(VELOCITY_PID_SLOT_ID, VELOCITY_KI, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.config_kD(VELOCITY_PID_SLOT_ID, VELOCITY_KD, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.configClosedloopRamp(CLOSED_LOOP_RAMP);
        leftTalonLead.configOpenloopRamp(0.2);

        rightTalonLead.config_kF(VELOCITY_PID_SLOT_ID, VELOCITY_KF, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.config_kP(VELOCITY_PID_SLOT_ID, VELOCITY_KP, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.config_kI(VELOCITY_PID_SLOT_ID, VELOCITY_KI, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.config_kD(VELOCITY_PID_SLOT_ID, VELOCITY_KD, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.configClosedloopRamp(CLOSED_LOOP_RAMP);
        rightTalonLead.configOpenloopRamp(0.2);

        // Configure Position PID
        leftTalonLead.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
        leftTalonLead.config_IntegralZone(POSITION_PID_SLOT_ID, 1000);
        leftTalonLead.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);

        rightTalonLead.config_kF(POSITION_PID_SLOT_ID, POSITION_KF, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.config_kP(POSITION_PID_SLOT_ID, POSITION_KP, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.config_kI(POSITION_PID_SLOT_ID, POSITION_KI, PID_CONFIG_TIMEOUT_MS);
        rightTalonLead.config_IntegralZone(POSITION_PID_SLOT_ID, 1000);
        rightTalonLead.config_kD(POSITION_PID_SLOT_ID, POSITION_KD, PID_CONFIG_TIMEOUT_MS);

        rightTalonLead.setNeutralMode(NeutralMode.Coast);
        rightTalonFollower.setNeutralMode(NeutralMode.Coast);
        leftTalonLead.setNeutralMode(NeutralMode.Coast);
        leftTalonFollower.setNeutralMode(NeutralMode.Coast);

        rightTalonLead.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        leftTalonLead.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);

        navX = new AHRS(SerialPort.Port.kMXP);

        m_kinematics = kinematics;

        leftTalonFollower.follow(leftTalonLead);
        rightTalonFollower.follow(rightTalonLead);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Put code here to be run every loop
        // Run Kinematics
        if (USE_NAVX_HEADING) {
            // use the NAVX for heading
            double headingInRadians = Math.toRadians(getGyroHeadingDegrees());
            m_kinematics.calculatePosition(getLeftEncoderPosition(), getRightEncoderPosition(), headingInRadians);
        } else {
            // Use the encoder information for heading
            m_kinematics.calculatePosition(getLeftEncoderPosition(), getRightEncoderPosition());
        }

        // Get current pose from Kinematics
        Position2D currentPose = m_kinematics.getPose();

        // Push robot info to Dashboard
        SmartDashboard.putNumber("Kinematics X (Feet)", currentPose.getX());
        SmartDashboard.putNumber("Kinematics Y (Feet)", currentPose.getY());
        SmartDashboard.putNumber("Kinematics Heading (degrees)", currentPose.getHeadingDegrees());

        SmartDashboard.putNumber("Left Encoder Velocity (Ft/S)", getLeftEncoderVelocity());
        SmartDashboard.putNumber("Left Encoder Position (Ft)", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Encoder Velocity (Ft/S)", getRightEncoderVelocity());
        SmartDashboard.putNumber("Right Encoder Position (Ft)", getRightEncoderPosition());
        SmartDashboard.putNumber("Raw Left Encoder", leftTalonLead.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Raw Right Encoder", rightTalonLead.getSelectedSensorPosition(0));

        SmartDashboard.putNumber("Robot Heading (degrees)", getGyroHeadingDegrees());
        SmartDashboard.putNumber("NavX X Accel", navX.getWorldLinearAccelX());
        SmartDashboard.putNumber("NavX Y Accel", navX.getWorldLinearAccelY());
        SmartDashboard.putNumber("NavX Z Accel", navX.getWorldLinearAccelZ());
        SmartDashboard.putNumber("NavX Yaw", getGyroYawDegrees());
        SmartDashboard.putNumber("NavX Angle", getGyroHeadingDegrees());
        SmartDashboard.putNumber("NavX Fused Heading", getGyroFusedHeadingDegrees());
        SmartDashboard.putNumber("NavX TurnRate dg/s", navX.getRate());

        SmartDashboard.putNumber("Left Motor Position Error", leftTalonLead.getClosedLoopError(0));
        SmartDashboard.putNumber("Right Motor Position Error", rightTalonLead.getClosedLoopError(0));
    }

    public void percentDrive(double leftPercentage, double rightPercentage) {
        leftTalonLead.set(ControlMode.PercentOutput, leftPercentage);
        rightTalonLead.set(ControlMode.PercentOutput, rightPercentage);
    }

    public void stop() {
        leftTalonLead.set(0);
        rightTalonLead.set(0);
    }

    // Sets the motors to encoder units per desisec (100ms), uses the onboard motor
    // PID
    public void velocityDrive(double vL, double vR) {
        SmartDashboard.putNumber("Set Velocity Left (Encoder units/100ms)", vL);
        SmartDashboard.putNumber("Set Velocity Right (Encoder units/100ms)", vR);
        leftTalonLead.selectProfileSlot(VELOCITY_PID_SLOT_ID, 0);
        rightTalonLead.selectProfileSlot(VELOCITY_PID_SLOT_ID, 0);
        leftTalonLead.set(TalonFXControlMode.Velocity, vL);
        rightTalonLead.set(TalonFXControlMode.Velocity, vR);
    }

    public void initAutonomous(Position2D startingPose) {
        // reset encoders
        zeroDriveTrainEncoders();

        m_kinematics.setPose(startingPose);
        // Reset Gyro
        setupGyro(navX, startingPose.getHeadingDegrees());
    }

    /**
     * returns left encoder position
     * 
     * @return left encoder position
     */
    public double getLeftEncoderPosition() {
        // Returns the number of steps, multiply by edges per step to get EPR, divided
        // by the gearbox ratio
        return SPIKE293Utils.controllerUnitsToFeet(leftTalonLead.getSelectedSensorPosition(0));
    }

    /**
     * returns right encoder position
     * 
     * @return right encoder position
     */
    public double getRightEncoderPosition() {
        // Returns the number of steps, multiply by edges per step to get EPR, divided
        // by the gearbox ratio
        return SPIKE293Utils.controllerUnitsToFeet(rightTalonLead.getSelectedSensorPosition(0));
    }

    /**
     * returns left encoder Velocity in ft/s
     * 
     * @return left encoder Velocity in ft/s
     */
    public double getLeftEncoderVelocity() {
        // Returns the velocity of encoder by claculating the velocity from encoder
        // units of click/100ms to ft/s
        return SPIKE293Utils.controllerVelocityToFeetPerSec(leftTalonLead.getSelectedSensorVelocity());
    }

    /**
     * returns right encoder Velocity in ft/s
     * 
     * @return right encoder Velocity in ft/s
     */
    public double getRightEncoderVelocity() {
        // Returns the velocity of encoder by claculating the velocity from encoder
        // units of click/100ms to ft/s
        return SPIKE293Utils.controllerVelocityToFeetPerSec(rightTalonLead.getSelectedSensorVelocity());
    }

    /**
     * returns robot Velocity in ft/s
     * 
     * @return robot Velocity in ft/s
     */
    public double getRobotVelocity() {
        // Returns the velocity of the robot by taking the averga of the velcity on both
        // sides of the robor
        return (getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2.0d;
    }

    /**
     * resets the drive train encoders to 0
     */
    private void zeroDriveTrainEncoders() {
        leftTalonLead.setSelectedSensorPosition(0);
        rightTalonLead.setSelectedSensorPosition(0);
    }

    public double getGyroFusedHeadingDegrees() {
        return (navX.getFusedHeading() * -1.0d);
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
        double leftEncoderPosition = leftTalonLead.getSelectedSensorPosition(0);
        double rightEncoderPosition = rightTalonLead.getSelectedSensorPosition(0);
        positionControl(leftEncoderPosition - encoderTicks, rightEncoderPosition + encoderTicks);
    }

    // sets left and right talons to given parameters
    public void positionControl(double posL, double posR) {
        leftTalonLead.selectProfileSlot(POSITION_PID_SLOT_ID, 0);
        rightTalonLead.selectProfileSlot(POSITION_PID_SLOT_ID, 0);

        leftTalonLead.set(ControlMode.Position, posL);
        rightTalonLead.set(ControlMode.Position, posR);
    }

    public double getMotorError() {
        return leftTalonLead.getClosedLoopError(0);
    }
}
