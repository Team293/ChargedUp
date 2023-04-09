package frc.robot.classes;

/**
 * This class is used to represent a position on the field,
 * or more generally, a position on a 2D plane, with an
 * associated heading.
 */
public class Position2D {
    private double m_x;
    private double m_y;
    private double m_heading; // In radians

    /**
     * Constructor for Position2D class.
     * 
     * @param x       x coordinate
     * @param y       y coordinate
     * @param heading heading in radians
     */
    public Position2D(double x, double y, double heading) {
        m_x = x;
        m_y = y;
        m_heading = heading;
    }

    /**
     * Sets the x coordinate of the Position.
     * 
     * @param x the x coordinate of the Position
     */
    public void setX(double x) {
        m_x = x;
    }

    /**
     * Sets the y coordinate of the Position.
     * 
     * @param y the y coordinate of the Position
     */
    public void setY(double y) {
        m_y = y;
    }

    /**
     * Sets the heading of the Position in degrees.
     * 
     * @param heading the heading of the Position in degrees
     */
    public void setHeadingDegrees(double heading) {
        m_heading = Math.toRadians(heading);
    }

    /**
     * Sets the heading of the Position in radians.
     * 
     * @param heading the heading of the Position in radians
     */
    public void setHeadingRadians(double heading) {
        m_heading = heading;
    }

    /**
     * Returns the x coordinate of the Position.
     * 
     * @return the x coordinate of the Position
     */
    public double getX() {
        return m_x;
    }

    /**
     * Returns the y coordinate of the Position.
     * 
     * @return the y coordinate of the Position
     */
    public double getY() {
        return m_y;
    }

    /**
     * Returns the heading of the Position in degrees.
     * 
     * @return the heading of the Position in degrees
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(m_heading);
    }

    /**
     * Returns the heading of the Position in radians.
     * 
     * @return the heading of the Position in radians
     */
    public double getHeadingRadians() {
        return m_heading;
    }
}
