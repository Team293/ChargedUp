package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.WriteToCSV;

import static frc.robot.Constants.AutonomousCommandConstants.*;

public class SequentialAutoCommand extends SequentialCommandGroup {
	private StartPositions m_startPosition;
	private Drivetrain m_drivetrain;
	private Kinematics m_kinematics;

	public SequentialAutoCommand(Drivetrain drivetrain, Kinematics kinematics, Targeting targeting,
			StartPositions startPosition, WriteToCSV logger) {
		m_drivetrain = drivetrain;
		m_kinematics = kinematics;
		m_startPosition = startPosition;

		SmartDashboard.putBoolean("AutoDone", false);

		switch (m_startPosition) { // Changes the robot path based on the starting position of the robot Left,
									// Middle, Right
			case BLUE_LEFT:
				Bottom();
				break;

			case BLUE_MIDDLE:
				Middle();
				break;

			case BLUE_RIGHT:
				Top();
				break;

			case RED_LEFT:
				Top();
				break;

			case RED_MIDDLE:
				Middle();
				break;

			case RED_RIGHT:
				Bottom();
				break;

			default:
				System.out.println("ERROR: Invalid autonomous starting position! [" + m_startPosition + "]");
				break;
		}

		// Alert smart dashboard that autonomous is done
		SmartDashboard.putBoolean("AutoDone", true);
	}

	private void Top() {
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(90)), m_drivetrain, m_kinematics),

				// Drive to the first ball and collect it
				new DriveTo(new Position2D(0, 8.4, Math.toRadians(90)), 2.0d, false, m_kinematics,
						m_drivetrain));
	}

	private void Middle() {
		addCommands(
				// Reset kinematics to the blue left position
			new DriveToBalance(m_drivetrain),
			new AutoBalance(m_drivetrain)
		);
	}

	private void Bottom() {
		addCommands(
				// Reset kinematics to the blue left position
				new ResetKinematics(new Position2D(0, 0, Math.toRadians(90)), m_drivetrain, m_kinematics),

				// Drive to the first ball and collect it
				new DriveTo(new Position2D(0, 15.1, Math.toRadians(90)), 2.0d, false, m_kinematics,
						m_drivetrain));
	}

}
