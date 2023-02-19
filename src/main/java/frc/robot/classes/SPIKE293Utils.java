package frc.robot.classes;

public final class SPIKE293Utils {
    public static final double SECONDS_TO_DECISEC = 1.0d / 10.0d;
    public static final double DECISEC_TO_SECONDS = 10.0d / 1.0d;
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
}
