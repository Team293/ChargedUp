package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;

import static frc.robot.Constants.AutonomousCommandConstants.*;

public class SequentialAutoCommand extends SequentialCommandGroup {
	private StartPositions m_startPosition;
	private Drivetrain m_drivetrain;
	private Kinematics m_kinematics;
	private Arm m_arm;
	private Claw m_claw;

	public SequentialAutoCommand(Drivetrain drivetrain, Arm arm, Claw claw, Kinematics kinematics, Targeting targeting,
			StartPositions startPosition) {
		m_drivetrain = drivetrain;
		m_arm = arm;
		m_claw = claw;
		m_kinematics = kinematics;
		m_startPosition = startPosition;

		RobotContainer.getAutoTab().setBoolean("AutoDone", false);

		switch (m_startPosition) { // Changes the robot path based on the starting position of the robot Left,
			// Middle, Right
			case BLUE_LEFT:
				bottom();
				break;

			case BLUE_MIDDLE:
				middle();
				break;

			case BLUE_RIGHT:
				top();
				break;

			case RED_LEFT:
				top();
				break;

			case RED_MIDDLE:
				middle();
				break;

			case RED_RIGHT:
				bottom();
				break;

			default:
				System.out.println("ERROR: Invalid autonomous starting position! [" + m_startPosition + "]");
				break;
		}

		// Alert smart dashboard that autonomous is done
		RobotContainer.getAutoTab().setBoolean("AutoDone", true);
	}

	private void top() {
		addCommands(
				new DriveToBalance(m_drivetrain));
		// Face Field
		// addCommands(
		// // Reset kinematics to the blue left position
		// new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain,
		// m_kinematics),
		// new SetClawForTime(m_claw, 1.0d, 1.0d),
		// // Close claw
		// new SetClaw(m_claw, -1.0d, 8.0d),
		// // Raise arm
		// new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.STOW_INCHES),
		// // Extend arm
		// new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.SCORE_HIGH_R_INCHES),
		// new Wait(3.0d),
		// // Drive forward (~1 foot)
		// new DriveTo(new Position2D(2, 0, Math.toRadians(0)), 1.0d, false,
		// m_kinematics, m_drivetrain),
		// new Wait(3.0d),
		// // Open claw
		// new SetClawForTime(m_claw, 1.0d, 1.0d),
		// new Wait(10.0d),
		// // Retract arm
		// new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.STOW_INCHES),
		// new Wait(10.0d),
		// // Drive backwards
		// new DriveBackwards(m_drivetrain, m_kinematics, -0.1d, 2.0d),
		// new Wait(10.0d),
		// // Lower arm
		// new SetArm(m_arm, MoveArm.STOW_ANGLE, MoveArm.STOW_INCHES)
		// // new SetArm(m_arm, MoveArm.STOW_ANGLE, MoveArm.STOW_INCHES)
		// );
	}

	// FACE CHARGE STATION
	private void middle() {
		double firstSpeed = RobotContainer.getAutoTab().getDouble("first speed", 0);
		firstSpeed = MathUtil.clamp(firstSpeed, -.4, 0);
		RobotContainer.getAutoTab().setDouble("first speed", firstSpeed);
		double firstDistance = RobotContainer.getAutoTab().getDouble("first distance", 0);
		firstDistance = MathUtil.clamp(firstDistance, 0, 10);
		RobotContainer.getAutoTab().setDouble("first distance", firstDistance);

		double secondSpeed = RobotContainer.getAutoTab().getDouble("second speed", 0);
		secondSpeed = MathUtil.clamp(secondSpeed, -.4, 0);
		RobotContainer.getAutoTab().setDouble("second speed", secondSpeed);
		double secondDistance = RobotContainer.getAutoTab().getDouble("second distance", 0);
		secondDistance = MathUtil.clamp(secondDistance, 0, 10);
		RobotContainer.getAutoTab().setDouble("second distance", secondDistance);

		double thirdSpeed = RobotContainer.getAutoTab().getDouble("third speed", 0);
		thirdSpeed = MathUtil.clamp(thirdSpeed, 0, 0.4);
		RobotContainer.getAutoTab().setDouble("third speed", thirdSpeed);
		double thirdDistance = RobotContainer.getAutoTab().getDouble("third distance", 0);
		thirdDistance = MathUtil.clamp(thirdDistance, -10, 10);
		RobotContainer.getAutoTab().setDouble("third distance", thirdDistance);

		addCommands(
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(180)), m_drivetrain,
						m_kinematics),
				new DriveBackwards(m_drivetrain, m_kinematics, firstSpeed, firstDistance),
				// new ResetKinematics(new Position2D(0, 0, Math.toRadians(180)), m_drivetrain,
				// m_kinematics),
				// new DriveBackwards(m_drivetrain, m_kinematics, -.15, 2),
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(180)), m_drivetrain,
						m_kinematics),
				new DriveBackwards(m_drivetrain, m_kinematics, secondSpeed, secondDistance),
				// new Wait(1),
				// Reset kinematics to the blue left position
				new DriveToBalance(m_drivetrain),
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(180)), m_drivetrain,
						m_kinematics),
				new DriveBackwards(m_drivetrain, m_kinematics, thirdSpeed, thirdDistance),
				new AutoBalance(m_drivetrain));
	}

	// FACE FIELD
	private void bottom() {
		// addCommands(
		// // Reset kinematics to the blue left position
		// new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain,
		// m_kinematics),

		// // Drive backwards for taxi auto points
		// new DriveTo(new Position2D(5, 0, Math.toRadians(0)), 0.5d, false,
		// m_kinematics, m_drivetrain),
		// new Wait(5.0d),
		// new DriveTo(new Position2D(-5, 0, Math.toRadians(0)), 0.5d, true,
		// m_kinematics, m_drivetrain));
	}

}
