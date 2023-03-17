package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
	private Arm m_arm;
	private Claw m_claw;
	private Kinematics m_kinematics;

	public SequentialAutoCommand(Drivetrain drivetrain, Arm arm, Claw claw,  Kinematics kinematics, Targeting targeting,
			StartPositions startPosition) {
		m_drivetrain = drivetrain;
		m_arm = arm;
		m_claw = claw;
		m_kinematics = kinematics;
		m_startPosition = startPosition;

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
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(90)), m_drivetrain, m_kinematics)
				// new MoveArm(m_arm, Node.HIGH),

				// // Drive to the first ball and collect it
				
				// Commands.race(new DriveTo(new Position2D(0, 8.4, Math.toRadians(90)), 1.5d, false, m_kinematics,
				// m_drivetrain), new CloseClaw(m_claw)), 

				// new OpenClaw(m_claw)
		);
				
	}

	private void middle() {
		addCommands(
				// Reset kinematics to the blue left position
			new DriveToBalance(m_drivetrain),
			new AutoBalance(m_drivetrain)
		);
	}

	private void bottom() {
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(90)), m_drivetrain, m_kinematics),

				// Drive to the first ball and collect it
				new DriveTo(new Position2D(0, 15.1, Math.toRadians(90)), 2.0d, false, m_kinematics,
						m_drivetrain));
	}

}
