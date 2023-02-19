package frc.robot.classes;

import static frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.MathUtil;

public final class SPIKE293Utils {

    public static final double MAX_ENCODER_VELOCITY = 20743.0d;

    public static final double WHEEL_CIRCUMFERENCE_FEET = (4.0d / 12.0d) * Math.PI; // Wheel radius 4 in, converting
                                                                                    // to feet
    public static final double SECONDS_TO_DECISEC = 1.0d / 10.0d;
    public static final double DECISEC_TO_SECONDS = 10.0d / 1.0d;
    public static final double GEARBOX_RATIO_TO_ONE = 9.52d;

    public static final int ENCODER_COUNTS_PER_REVOLUTION = 2048;
    public static final int ENCODER_UNITS_PER_REVOLUTION = ENCODER_COUNTS_PER_REVOLUTION; // Edges per Rotation

    public static final double GEAR_RATIO = 1.0d;
    public static final double MINUTES_TO_DECISECONDS = 600.0d;

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
        } else {
            retval = 0;
        }

        return retval;
    }

    /**
     * Converts percentage to encoder velocity
     * 
     * @param percentage
     * @return velocity in encoder units(edges per 100 milliseconds) for drivetrain
     */
    public static double percentageToControllerVelocity(double percentage) {
        MathUtil.clamp(percentage, -1.0d, 1.0d);
        return (percentage * DrivetrainConstants.MAX_ENCODER_VELOCITY);
    }
}
