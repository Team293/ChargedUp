package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;

public class SequentialAutoCommand extends SequentialCommandGroup {
    public static enum StartPositions {
        DONT_MOVE,
        DRIVE_FORWARD,
        WALL_SIDE_SCORE,
        CENTER_ENGAGE,
        SUBSTATION_SIDE_SCORE
	}

	private StartPositions m_startPosition;
	private Drivetrain m_drivetrain;
	// private Arm m_arm;
	// private Claw m_claw;
	private Kinematics m_kinematics;
	private Arm m_arm;
	private Claw m_claw;

	public SequentialAutoCommand(Drivetrain drivetrain, Arm arm, Claw claw, Kinematics kinematics, Targeting targeting,
			StartPositions startPosition) {
		m_drivetrain = drivetrain;
		// m_arm = arm;
		// m_claw = claw;
		m_kinematics = kinematics;
		m_startPosition = startPosition;
		m_arm = arm;
		m_claw = claw;

		SmartDashboard.putBoolean("AutoDone", false);

		switch (m_startPosition) { // Changes the robot path based on the starting position of the robot Left,
			// Middle, Right

			case SUBSTATION_SIDE_SCORE:
				substationSide();
				break;

			case CENTER_ENGAGE:
				chargeStationCenter();
				break;

			case WALL_SIDE_SCORE:
				wallSide();
				break;

			default:
				System.out.println("ERROR: Invalid autonomous starting position! [" + m_startPosition + "]");
				break;
		}

		// Alert smart dashboard that autonomous is done
		SmartDashboard.putBoolean("AutoDone", true);
	}

	private void wallSide() {
		// Face scoring grid
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics),
				new SetClawForTime(m_claw, 1.0d, 1.0d),
				// Close claw
				new SetClaw(m_claw, -1.0d, 8.0d),
				// Raise arm
				new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.STOW_INCHES),
				// Extend arm
				new SetArm(m_arm,  MoveArm.SCORE_HIGH_ANGLE, MoveArm.SCORE_HIGH_R_INCHES),
				new Wait(3.0d),
				// Drive forward (~1 foot)
				new DriveTo(new Position2D(2, 0, Math.toRadians(0)), 1.0d, m_kinematics, m_drivetrain),
				new Wait(3.0d),
				// Open claw
				new SetClawForTime(m_claw, 1.0d, 1.0d),
				new Wait(3.0d),
				// Retract arm
				new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.STOW_INCHES),
				new Wait(3.0d),
				// Drive backwards
				new DriveTo(new Position2D(-10, 1, Math.toRadians(0)), -1.0d, m_kinematics, m_drivetrain),
				new Wait(3.0d),
				// Lower arm
				new SetArm(m_arm, MoveArm.STOW_ANGLE, MoveArm.STOW_INCHES)
				// new SetArm(m_arm, MoveArm.STOW_ANGLE, MoveArm.STOW_INCHES)
		);
	}

	// FACE CHARGE STATION
	private void chargeStationCenter() {
		// Accept master branch changes
		throw new UnsupportedOperationException("Accept master branch changes.");			
	}

	// FACE FIELD
	private void substationSide() {
		// Face scoring grid
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics),
				new SetClawForTime(m_claw, 1.0d, 1.0d),
				// Close claw
				new SetClaw(m_claw, -1.0d, 8.0d),
				// Raise arm
				new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.STOW_INCHES),
				// Extend arm
				new SetArm(m_arm,  MoveArm.SCORE_HIGH_ANGLE, MoveArm.SCORE_HIGH_R_INCHES),
				new Wait(3.0d),
				// Drive forward (~1 foot)
				new DriveTo(new Position2D(2, 0, Math.toRadians(0)), 1.0d, m_kinematics, m_drivetrain),
				new Wait(3.0d),
				// Open claw
				new SetClawForTime(m_claw, 1.0d, 1.0d),
				new Wait(3.0d),
				// Retract arm
				new SetArm(m_arm, MoveArm.SCORE_HIGH_ANGLE, MoveArm.STOW_INCHES),
				new Wait(3.0d),
				// Drive backwards
				new DriveTo(new Position2D(-10, 1, Math.toRadians(0)), -1.0d, m_kinematics, m_drivetrain),
				new Wait(3.0d),
				// Lower arm
				new SetArm(m_arm, MoveArm.STOW_ANGLE, MoveArm.STOW_INCHES)
				// new SetArm(m_arm, MoveArm.STOW_ANGLE, MoveArm.STOW_INCHES)
		);
	}

}
