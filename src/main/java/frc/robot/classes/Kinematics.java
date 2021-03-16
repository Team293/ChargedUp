package frc.robot.classes;

//import frc.robot.classes.Position2D;
import static frc.robot.Constants.DrivetrainConstants.*;

public class Kinematics
{
  private Position2D m_currentPose;
  private double m_oldLeftDistanceTraveled;
  private double m_oldRightDistanceTraveled;

  public Kinematics(Position2D startingPosition)
  {
    m_currentPose = startingPosition;
    m_oldLeftDistanceTraveled = 0;
    m_oldRightDistanceTraveled = 0;
  }

  public void calculatePosition(double leftEncoderPostion, double rightEncoderPosition)
  {
    double deltaLeft = 0.0d;
    double deltaRight = 0.0d;
    double deltaPsi = 0.0d;
    double deltaX = 0.0d;
    double deltaY = 0.0d;

    //Calculate distance traveled
    deltaLeft = leftEncoderPostion - m_oldLeftDistanceTraveled;
    deltaRight = rightEncoderPosition - m_oldRightDistanceTraveled;

    //Save distance traveled
    m_oldLeftDistanceTraveled = leftEncoderPostion;
    m_oldRightDistanceTraveled = rightEncoderPosition;

    //Calculate new attitude angle in radians
    deltaPsi = (deltaRight - deltaLeft) / TRACK_WIDTH_FEET; //this is the width from center left wheel to center right wheel

    //Calcualte new X, Y
    deltaX = ((deltaLeft + deltaRight)/2) * Math.cos(m_currentPose.getHeadingRadians());    //previous heading is in radians
    deltaY = ((deltaLeft + deltaRight)/2) * Math.sin(m_currentPose.getHeadingRadians());    //previous heading is in radians

    //Update new position
    m_currentPose.setX(m_currentPose.getX() + deltaX);
    m_currentPose.setY(m_currentPose.getY() + deltaY);

    //Limit psi between Pi and -Pi
    deltaPsi = limitRadians(deltaPsi);

    m_currentPose.setHeadingRadians(m_currentPose.getHeadingRadians() + deltaPsi);
  }

  public Position2D getPose()
  {
    return m_currentPose;
  }

  private double limitRadians(double radians)
  {
    double retval = radians;
    
    while(retval > Math.PI)
    {
      retval -= 2 * Math.PI;
    }

    while(retval < -Math.PI)
    {
      retval += 2 * Math.PI;
    }

    return retval;
  }
}
