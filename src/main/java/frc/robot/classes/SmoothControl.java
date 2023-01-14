package frc.robot.classes;

import static frc.robot.Constants.SmoothControlConstants.*;

public class SmoothControl {
  private double m_range; // Feet

  public SmoothControl() {
    m_range = 0.0d;
  }

  // Modification of equation 13, Calculates the omegaDesired (in radians)
  // modified by velocity aggressivness given required turn rate
  public double computeTurnRate(Position2D currentPose, Position2D targetPose, double maxVelocity) {
    double poseHeading = currentPose.getHeadingRadians(); // Convert heading to radians

    // Limit pose heading to be within -Pi and Pi
    poseHeading = limitRadians(poseHeading); // poseHeading is now radians

    // With the velocity and robot heading set appropriately, get the range
    // and the vector orientation that runs from the robot to the target
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double range = Math.sqrt(dx * dx + dy * dy); // distance in feet
    double r_angle = Math.atan2(dy, dx); // vector heading in radians

    // Compute the angle between this vector and the desired orientation at the
    // target
    double thetaT = targetPose.getHeadingRadians() - r_angle;
    thetaT = limitRadians(thetaT); // bound this between -PI to PI

    // Compute the angle between current robot heading and the vector from
    // the robot to the target
    double del_r = poseHeading - r_angle;
    del_r = limitRadians(del_r); // bound this between -PI to PI

    // Calculate k (equation 14)
    double k = k(range, thetaT, del_r);

    // All set, now the equation for the angular rate!
    double omegaDesired = vGivenK(k, maxVelocity) * k;

    // Update m_range
    m_range = range;

    return (omegaDesired);
  }

  // Equation 14, Calculates the required turn rate
  private double k(double range, double theta, double delta) {
    return (-(1 / m_range) * (K2 * (delta - Math.atan(-K1 * theta)) +
        Math.sin(delta) * (1.0 + (K1 / (1.0 + Math.pow((K1 * theta), 2))))));
  }

  // Equation 15, Calculates how close we are to maximum velocity based off of
  // required turn rate
  private double vGivenK(double k, double vMax) {
    double beta = 0.4d;
    double lambda = 2;
    return (vMax / (1 + Math.abs(Math.pow((beta * k), lambda))));
  }

  public double limitRadians(double radians) {
    double retval = radians;

    while (retval > Math.PI) {
      retval -= 2 * Math.PI;
    }

    while (retval < -Math.PI) {
      retval += 2 * Math.PI;
    }

    return retval;
  }

  public double getRange() {
    return m_range;
  }
}
