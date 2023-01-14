package frc.robot.classes;
import static frc.robot.Constants.DrivetrainConstants.*;
import edu.wpi.first.math.MathUtil;

public final class SPIKE293Utils 
{
    private SPIKE293Utils() 
    {
        throw new AssertionError("utility class");
    }

    public static double applyDeadband(double input, double deadband)
    {
        double retval = input;
        if (Math.abs(input) >= deadband)
        {
            retval = (Math.abs(input) - deadband)/(1 - deadband);
            //Check if value is supposed to be negative
            if(input < 0)
            {
                retval = -1 * retval;
            } 
        }

        return retval;
    }

    // Converts from encoder edges per 100 milliseconds to feet per second.
    public static double controllerVelocityToFeetPerSec(double encoderUnits) 
    {
        return controllerUnitsToFeet(encoderUnits) * DECISEC_TO_SECONDS;
    }

    //Converts from feet per second to encoder edges per 100 milliseconds.
    public static double feetPerSecToControllerVelocity(double feetPerSec) 
    {
        return ((feetPerSec  * GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION) / (WHEEL_CIRCUMFERENCE_FEET)) * SECONDS_TO_DECISEC;
    }

    // Converts from encoder units to feet
    public static double controllerUnitsToFeet(double encoderUnits) 
    {
        return (encoderUnits *  WHEEL_CIRCUMFERENCE_FEET) / (GEARBOX_RATIO_TO_ONE * ENCODER_UNITS_PER_REVOLUTION);
    }

    //Converts percentage to encoder velocity
    public static double percentageToControllerVelocity(double percentage)
    {
        MathUtil.clamp(percentage, -1.0d , 1.0d);
        return (percentage * MAX_ENCODER_VELOCITY);
    }
}
