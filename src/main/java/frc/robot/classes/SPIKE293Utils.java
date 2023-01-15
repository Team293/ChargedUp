package frc.robot.classes;

import static frc.robot.Constants.DrivetrainConstants;
import static frc.robot.Constants.LauncherConstants;
import edu.wpi.first.math.MathUtil;

public final class SPIKE293Utils {
    private SPIKE293Utils() {
        throw new AssertionError("utility class");
    }

    /**
     * 
     * @param input
     * @param deadband
     * @return deadband applied output
     */
    public static double applyDeadband(double input, double deadband) {
        double retval = input;
        if (Math.abs(input) >= deadband) {
            retval = (Math.abs(input) - deadband) / (1 - deadband);
            // Check if value is supposed to be negative
            if (input < 0) {
                retval = -1 * retval;
            }
        }

        return retval;
    }

    /**
    * Converts from encoder edges per 100 milliseconds to feet per second.
    *@return Drivetrain velocity in ft/s from encoder units
    */
    public static double controllerVelocityToFeetPerSec(double encoderUnits) {
        return controllerUnitsToFeet(encoderUnits) * DrivetrainConstants.DECISEC_TO_SECONDS;
    }

    /**
    * Converts from encoder edges per 100 milliseconds to feet per second.
    *@return Drivetrain velocity in ft/s from encoder units
    */
    public static double controllerVelocityToFeetPerSec(double encoderUnits, double wheelDiameter) {
        return controllerUnitsToFeet(encoderUnits, wheelDiameter) * DrivetrainConstants.DECISEC_TO_SECONDS;
    }

    /**
     * Converts from feet per second to encoder edges per 100 milliseconds.
     * @param Speed in ft/s
     * 
     * @return Drivetrain velocity in encoder units(edges per 100 milliseconds)
     */
    public static double feetPerSecToControllerVelocity(double feetPerSec) {
        return ((feetPerSec * DrivetrainConstants.GEARBOX_RATIO_TO_ONE * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION) / (DrivetrainConstants.WHEEL_CIRCUMFERENCE_FEET))
                * DrivetrainConstants.SECONDS_TO_DECISEC;
    }

    /**
     * Converts from feet per second to encoder edges per 100 milliseconds.
     * @param Speed in ft/s
     * @param Wheel diameter in ft
     * 
     * @return Drivetrain velocity in encoder units(edges per 100 milliseconds)
     */
    public static double feetPerSecToControllerVelocity(double feetPerSec, double wheelDiameter) {
        return ((feetPerSec * DrivetrainConstants.GEARBOX_RATIO_TO_ONE * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION) / (wheelDiameter * Math.PI))
                * DrivetrainConstants.SECONDS_TO_DECISEC;
    }

    /**
     * Converts from feet per second to encoder edges per 100 milliseconds.
     * @param Speed in ft/s
     * @param Wheel diameter in ft
     * @param Gearbox ratio
     * 
     * @return Drivetrain velocity in encoder units(edges per 100 milliseconds)
     */
    public static double feetPerSecToControllerVelocity(double feetPerSec, double wheelDiameter, double gearboxRatio) {
        return ((feetPerSec * gearboxRatio * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION) / (wheelDiameter * Math.PI))
                * DrivetrainConstants.SECONDS_TO_DECISEC;
    }

    /**
     * 
     * @param Encoder Units
     * @return feet from encoder units for drivetrain
     */
    public static double controllerUnitsToFeet(double encoderUnits) {
        return (encoderUnits * DrivetrainConstants.WHEEL_CIRCUMFERENCE_FEET) / (DrivetrainConstants.GEARBOX_RATIO_TO_ONE * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION);
    }

    /**
     * 
     * @param Encoder Units
     * @param Wheel diameter in ft
     * @return feet from encoder units for drivetrain
     */
    public static double controllerUnitsToFeet(double encoderUnits, double wheelDiameter) {
        return (encoderUnits * wheelDiameter * Math.PI) / (DrivetrainConstants.GEARBOX_RATIO_TO_ONE * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION);
    }

    /**
     * 
     * @param feet
     * @return encoder units from feet for drivetrain
     */
    public static double feetToControllerUnits(double feet) {
        double controllerUnits = 0.0d;
        controllerUnits = ((feet * DrivetrainConstants.GEARBOX_RATIO_TO_ONE * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION) / (DrivetrainConstants.WHEEL_CIRCUMFERENCE_FEET));
        return controllerUnits;
    }

    /**
     * 
     * @param feet
     * @param Wheel diameter in ft
     * @return encoder units from feet for drivetrain
     */
    public static double feetToControllerUnits(double feet, double wheelDiameter) {
        double controllerUnits = 0.0d;
        controllerUnits = ((feet * DrivetrainConstants.GEARBOX_RATIO_TO_ONE * DrivetrainConstants.ENCODER_UNITS_PER_REVOLUTION) / (wheelDiameter * Math.PI));
        return controllerUnits;
    }

    /**
     * Converts percentage to encoder velocity
     * @param percentage
     * @return velocity in encoder units(edges per 100 milliseconds) for drivetrain
     */
    public static double percentageToControllerVelocity(double percentage) {
        MathUtil.clamp(percentage, -1.0d, 1.0d);
        return (percentage * DrivetrainConstants.MAX_ENCODER_VELOCITY);
    }

    /**
     * Converts RPMs to encoder units per desiseconds (100ms)
     * @param rpm
     * @return velocity in encoder units (edges per 100) of shooter wheel
     */
    public static double convertRPMToControllerVelocity(double rpm) {
        return (rpm * LauncherConstants.GEAR_RATIO * LauncherConstants.ENCODER_UNITS_PER_REVOLUTION) / LauncherConstants.MINUTES_TO_DECISECONDS;
    }

    /**
     * Converts Encoder units per desiseconds (100ms) to RPMs
     * @param velocity (encoder units per 100 milliseconds)
     * @return rpm of shooter wheel
     */
    public static double convertControllerVelocityToRPM(double velocity) {
        return (velocity * LauncherConstants.MINUTES_TO_DECISECONDS) / (LauncherConstants.ENCODER_UNITS_PER_REVOLUTION * LauncherConstants.GEAR_RATIO);
    }

    /**
     * Converts RPMs to ft/s
     * @param rpm
     * @param Wheel diameter in ft
     * @return velocity in ft/s
     */
    public static double convertRPMToFeetPerSec(double rpm, double wheelDiameter) {
        return rpm * Math.PI * wheelDiameter / DrivetrainConstants.SECONDS_PER_MINUTE;
    }

    /**
     * Converts ft/s to RPMs
     * @param velocity in ft/s
     * @param Wheel diameter in ft
     * @return rpm
     */
    public static double convertFeetPerSecToRPM(double velocity, double wheelDiameter) {
        return velocity * DrivetrainConstants.SECONDS_PER_MINUTE / (Math.PI * wheelDiameter);
    }

    /**
     * Convert rotations to feet
     * @param rotations
     * @param Wheel diameter in ft
     * @return feet
     */
    public static double convertRotationsToFeet(double rotations, double wheelDiameter) {
        return rotations * wheelDiameter * Math.PI;
    }

    /**
     * Convert feet to rotations
     * @param feet
     * @param Wheel diameter in ft
     * @return rotations
     */
    public static double convertFeetToRotations(double feet, double wheelDiameter) {
        return feet / (wheelDiameter * Math.PI);
    }
}
