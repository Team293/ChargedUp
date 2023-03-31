package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        DRIVE_BACKWARD,
        WALL_SIDE_SCORE,
        CENTER_ENGAGE,
        SUBSTATION_SIDE_SCORE,
		SCORE_DONT_MOVE
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

		switch (m_startPosition) { // Changes the robot path based on the starting position of the robot
			case SCORE_DONT_MOVE:
				// FACE SCORING GRID
				resetKinematics();
				score();
				addCommands(
					// Lower arm
					new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES)
				);
				break;

			case SUBSTATION_SIDE_SCORE:
				// FACE SCORING GRID
				resetKinematics();
				score();
				addCommands(
					// Drive backwards and lower arm
					new ParallelCommandGroup(
						new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES),
						new DriveTo(new Position2D(-10, 1, Math.toRadians(0)), -5.0d, m_kinematics, m_drivetrain)
					)
				);
				break;

			case CENTER_ENGAGE:
				// FACE SCORING GRID
				resetKinematics();
				chargeStationCenter();
				break;

			case WALL_SIDE_SCORE:
				// FACE SCORING GRID
				resetKinematics();
				score();
				addCommands(
					// Drive backwards and lower arm
					new ParallelCommandGroup(
						new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES),
						new DriveTo(new Position2D(-10, -1, Math.toRadians(0)), -5.0d, m_kinematics, m_drivetrain)
					)
				);
				break;

			case DRIVE_BACKWARD:
				// FACE SCORING GRID
				resetKinematics();
				addCommands(
					// Drive backwards
					new DriveTo(new Position2D(-10, 0, Math.toRadians(0)), -5.0d, m_kinematics, m_drivetrain)
				);
				break;

			default:
				resetKinematics();
				System.out.println("ERROR: Invalid autonomous starting position! [" + m_startPosition + "]");
				break;
		}

		// Alert smart dashboard that autonomous is done
		SmartDashboard.putBoolean("AutoDone", true);
	}

	// Score a piece in the high node
	public void score() {
		addCommands(
			new SetClawForTime(m_claw, 1.0d, 1.0d),
			// Close claw
			new SetClaw(m_claw, -1.0d, 10.0d),
			// Raise arm
			new SetArm(m_arm, MoveArm.HIGH_ANGLE, Arm.STOW_R_INCHES),
			new Wait(1.0d),
			// Extend arm
			new SetArm(m_arm,  MoveArm.HIGH_ANGLE, MoveArm.HIGH_R_INCHES),
			// Drive forward (~1 foot)
			new DriveTo(new Position2D(2, 0, Math.toRadians(0)),2.0d, m_kinematics, m_drivetrain),
			// Open claw
			new SetClawForTime(m_claw, 1.0d, 10.0d),
			new Wait(0.5d),
			// Retract arm
			new SetArm(m_arm, MoveArm.HIGH_ANGLE, Arm.STOW_R_INCHES),
			// Drive backwards
			new DriveTo(new Position2D(-0.5, 0, Math.toRadians(0)), -1.5d, m_kinematics, m_drivetrain)
		);
	}

	// FACE CHARGE STATION
	private void chargeStationCenter() {
		// Accept master branch changes
		throw new UnsupportedOperationException("Accept master branch changes.");			
	}

	private void resetKinematics() {
		addCommands(
			// Reset kinematics to the blue left position
			new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics)
		);
	}
}
