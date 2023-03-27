package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;
import frc.robot.commands.MoveArm.Node;
import frc.robot.commands.SetArm;

import static frc.robot.Constants.AutonomousCommandConstants.*;

public class SequentialAutoCommand extends SequentialCommandGroup {
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
		SmartDashboard.putBoolean("AutoDone", true);
	}

	private void top() {
		//Face Field
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics),
				// Close claw
				new SetClaw(m_claw, -1.0d, 10.0d),
				// Raise arm
				new SetArm(m_arm, MoveArm.SCORE_HYBRID_ANGLE, m_arm.getRInches()),
				// Extend arm
                new SetArm(m_arm, m_arm.getTheta(), MoveArm.SCORE_HIGH_ANGLE),
				// Drive forward (~1 foot)
				new DriveTo(new Position2D(1, 0, Math.toRadians(0)), 2.0d, false, m_kinematics, m_drivetrain),
				// Open claw
				new SetClaw(m_claw, -1.0d, 10.0d)
				// Back out of community
				// new DriveTo(new Position2D(6, 0, Math.toRadians(0)), 2.0d, true, m_kinematics, m_drivetrain)
				// Retract arm

				// Lower arm

				// driving
				// new DriveTo(new Position2D(6, 0, Math.toRadians(0)), 2.0d, false, m_kinematics, m_drivetrain)
				);
	}

    // FACE CHARGE STATION
	private void middle() {
		addCommands(
				// Reset kinematics to the blue left position
				new DriveToBalance(m_drivetrain),
				new AutoBalance(m_drivetrain)
            );
	}

    // FACE FIELD
	private void bottom() {
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics),

				// Drive backwards for taxi auto points
				new DriveTo(new Position2D(7, 0, Math.toRadians(0)), 2.0d, false, m_kinematics, m_drivetrain));
	}

}
