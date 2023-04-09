package frc.robot.classes;

import frc.robot.subsystems.Drivetrain;

/**
 * This class is used to calculate the current position of the robot based on
 * encoder readings and current heading.
 */
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

	/**
	 * Calculates the current position of the robot based on encoder readings and
	 * current heading.
	 * @param leftEncoderPosition  The current position of the left encoder.
	 * @param rightEncoderPosition The current position of the right encoder.
	 * @param heading              The current heading of the robot.
	 */
	public void calculatePosition(double leftEncoderPostion, double rightEncoderPosition, double heading) {
		double currentHeading = 0.0d;

		updateCurrentPose(leftEncoderPostion, rightEncoderPosition);

		// Limit psi between Pi and -Pi
		currentHeading = limitRadians(heading);

		// Calculate new attitude angle in radians
		m_currentPose.setHeadingRadians(currentHeading);
	}

	/**
	 * This function updates the robot's current pose based off of encoder distance traveled
	 * @param leftEncoderPosition distance traveled by the left encoder
	 * @param rightEncoderPosition distance traveled by the right encoder
	 */
	public void calculatePosition(double leftEncoderPostion, double rightEncoderPosition) {
		updateCurrentPose(leftEncoderPostion, rightEncoderPosition);
	}

	/**
	 * Updates the current pose of the robot based on the left and right encoder positions.
	 * @param leftEncoderPosition the left encoder position in feet
	 * @param rightEncoderPosition the right encoder position in feet
     */
	private void updateCurrentPose(double leftEncoderPostion, double rightEncoderPosition) {
		double deltaLeft = 0.0d;
		double deltaRight = 0.0d;
		double deltaX = 0.0d;
		double deltaY = 0.0d;
		double deltaPsi = 0.0d;
		double newPsi = 0.0d;

		// Calculate distance traveled
		deltaLeft = (leftEncoderPostion - m_previousLeftEncoderPosition);
		deltaRight = (rightEncoderPosition - m_previousRightEncoderPosition);

		// Save distance traveled
		m_previousLeftEncoderPosition = leftEncoderPostion;
		m_previousRightEncoderPosition = rightEncoderPosition;

		// Use encoders to calculate heading
		deltaPsi = (deltaRight - deltaLeft) / m_trackWidthFeet; // this is the width from center left wheel to center
																// right
																// wheel

		// Limit psi between Pi and -Pi
		deltaPsi = limitRadians(deltaPsi);
		newPsi = limitRadians(m_currentPose.getHeadingRadians() + deltaPsi);

		// Calcualte new X, Y
		deltaX = ((deltaLeft + deltaRight) / 2) * Math.cos(m_currentPose.getHeadingRadians()); // previous heading is in
																								// radians
		deltaY = ((deltaLeft + deltaRight) / 2) * Math.sin(m_currentPose.getHeadingRadians()); // previous heading is in
																								// radians

		// Update new position
		m_currentPose.setX(m_currentPose.getX() + deltaX);
		m_currentPose.setY(m_currentPose.getY() + deltaY);
		m_currentPose.setHeadingRadians(newPsi);
	}

	/**
	 * Returns the current pose of the robot as a Position2D object, containing the
	 * X, Y, and heading of the robot.
	 * @return the current pose of the robot.
	 */
	public Position2D getPose() {
		return m_currentPose;
	}

	/**
	 * Sets the current pose of the robot.
	 * @param pose the new pose of the robot.
	 */
	public void setPose(Position2D pose) {
		// Reset Post
		m_currentPose = pose;
	}

	/**
	 * Limits the angle between -Pi and Pi.
	 * @param radians the angle in radians.
	 * @return the limited angle in radians.
	 */
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

	/**
	 * Calculates the distance between the current pose and a checkpoint.
	 * @param checkpoint the checkpoint to calculate the distance to.
	 * @return the distance to the checkpoint.
	 */
	public double distanceFromPose(Position2D checkpoint) {
		double x1 = m_currentPose.getX();
		double y1 = m_currentPose.getY();
		double x2 = checkpoint.getX();
		double y2 = checkpoint.getY();
		double distance = Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));

		return distance;
	}

	/**
	 * Checks if the robot is at a checkpoint.
	 * @param checkpoint the checkpoint to check.
	 * @param threshold the threshold distance to check if the robot is at the checkpoint.
	 * @return true if the robot is at the checkpoint, false otherwise.
	 */
	public boolean atPose(Position2D checkpoint, double threshold) {
		double distanceFromPose = distanceFromPose(checkpoint);
		if (distanceFromPose <= threshold) {
			return true;
		} else {
			return false;
		}
	}
}
