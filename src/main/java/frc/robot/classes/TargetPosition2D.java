package frc.robot.classes;

public class TargetPosition2D extends Position2D
{
  double m_velocity;

  public TargetPosition2D(double x, double y, double heading, double velocity)
  {
    super(x, y, heading);
    m_velocity = velocity;
  }

  public double getVelocity()
  {
    return m_velocity;
  }
}
