
public class App {

    public static final double WITHIN_RANGE_MODIFIER = 1.0d/4.0d;

    private Kinematics m_kinematics;
    private Position2D m_targetPose;
    private SmoothControl m_smoothControl;
    
    private double m_maxVelocity;
    private boolean m_inReverse = false;
    private boolean m_isDone = false;

    public static void main(String[] args) throws Exception {
        do {
            
        } while (condition);
        

    }

    public double[] execute(){
        double vR = 0.0;
        double vL = 0.0;
        final double trackWidthFeet = 2.292;

        // Start auto nav drive routine
        if (true == m_inReverse) {
            // We're in reverse, heading needs to be reversed for smooth control algorithm
            m_targetPose.setHeadingRadians(m_targetPose.getHeadingRadians() + Math.PI);
        }

        // Compute turn rate in radians and update range
        double omegaDesired = m_smoothControl.computeTurnRate(m_kinematics.getPose(), m_targetPose, m_maxVelocity);

        if (true == m_inReverse) {
            // Calculate vR in feet per second
            vR = -m_maxVelocity - (trackWidthFeet / 2) * omegaDesired;
            // Calculate vL in feet per second
            vL = -m_maxVelocity + (trackWidthFeet / 2) * omegaDesired;
        } else {
            // Calculate vR in feet per second
            vR = m_maxVelocity + (trackWidthFeet / 2) * omegaDesired;
            // Calculate vL in feet per second
            vL = m_maxVelocity - (trackWidthFeet / 2) * omegaDesired;
        }

        // Converting ft/s equation output to controller velocity
        vR = SPIKE293Utils.feetPerSecToControllerVelocity(vR);
        vL = SPIKE293Utils.feetPerSecToControllerVelocity(vL);

        // Send vR and vL to velocity drive, units are in controller velocity
        

        // Have we reached the target?
        if ((trackWidthFeet * WITHIN_RANGE_MODIFIER) >= m_smoothControl.getRange()) {
            // ending the command to allow the next sequential command with next point to
            // run
            m_isDone = true;
        }

        return new double[]{vR, vL};

    }
}
