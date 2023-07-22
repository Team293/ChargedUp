package frc.robot.classes;

import frc.robot.subsystems.Drivetrain;

public class Kinematics {
  private Position2D m_currentPose;
  private double m_previousLeftEncoderPosition;
  private double m_previousRightEncoderPosition;
  private double m_trackWidthFeet = Drivetrain.TRACK_WIDTH_FEET;
  
  public Kinematics(Position2D startingPosition) {
    m_currentPose = startingPosition;
    m_previousLeftEncoderPosition = 0;
    m_previousRightEncoderPosition = 0;
  }

  // This function runs the kinematics calculation,
  // given the delta Psi to use instead fo calculating from encoders
  public void calculatePosition(double leftEncoderPosition, double rightEncoderPosition, double heading) {
    double currentHeading = 0.0d;

    updateCurrentPose(leftEncoderPosition, rightEncoderPosition);

    // Limit psi between Pi and -Pi
    currentHeading = limitRadians(heading);

    // Calculate new attitude angle in radians
    m_currentPose.setHeadingRadians(currentHeading);
  }

  // This function runs the kinematics calculation,
  // calculating delta Psi based off of encoder distance traveled
  public void calculatePosition(double leftEncoderPosition, double rightEncoderPosition) {
    updateCurrentPose(leftEncoderPosition, rightEncoderPosition);
  }

  private void updateCurrentPose(double leftEncoderPosition, double rightEncoderPosition) {
    double deltaLeft = 0.0d;
    double deltaRight = 0.0d;
    double deltaX = 0.0d;
    double deltaY = 0.0d;
    double deltaPsi = 0.0d;
    double newPsi = 0.0d;

    // Calculate distance traveled
    deltaLeft = (leftEncoderPosition - m_previousLeftEncoderPosition);
    deltaRight = (rightEncoderPosition - m_previousRightEncoderPosition);

    // Save distance traveled
    m_previousLeftEncoderPosition = leftEncoderPosition;
    m_previousRightEncoderPosition = rightEncoderPosition;

    // Use encoders to calculate heading
    deltaPsi = (deltaRight - deltaLeft) / m_trackWidthFeet; // this is the width from center left wheel to center right
                                                            // wheel

    // Limit psi between Pi and -Pi
    deltaPsi = limitRadians(deltaPsi);
    newPsi = limitRadians(m_currentPose.getHeadingRadians() + deltaPsi);

    // Calculate new X, Y
    deltaX = ((deltaLeft + deltaRight) / 2) * Math.cos(m_currentPose.getHeadingRadians()); // previous heading is in
                                                                                           // radians
    deltaY = ((deltaLeft + deltaRight) / 2) * Math.sin(m_currentPose.getHeadingRadians()); // previous heading is in
                                                                                           // radians

    // Update new position
    m_currentPose.setX(m_currentPose.getX() + deltaX);
    m_currentPose.setY(m_currentPose.getY() + deltaY);
    m_currentPose.setHeadingRadians(newPsi);
  }

  public Position2D getPose() {
    return m_currentPose;
  }

  public void setPose(Position2D pose) {
    // Reset Post
    m_currentPose = pose;
  }

  private double limitRadians(double radians) {
    double retval = radians;

    while (retval > Math.PI) {
      retval -= 2 * Math.PI;
    }

    while (retval < -Math.PI) {
      retval += 2 * Math.PI;
    }

    return retval;
  }

  public double distanceFromPose(Position2D checkpoint) {
    double x1 = m_currentPose.getX();
    double y1 = m_currentPose.getY();
    double x2 = checkpoint.getX();
    double y2 = checkpoint.getY();
    return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
  }

  public boolean atPose(Position2D checkpoint, double threshold) {
    double distanceFromPose = distanceFromPose(checkpoint);
    return distanceFromPose <= threshold;
  }
}
