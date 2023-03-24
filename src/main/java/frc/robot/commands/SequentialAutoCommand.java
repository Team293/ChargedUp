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
    private Kinematics m_kinematics;
    private AutoTarget m_autoBall;
    private Targeting m_targeting;
    private double m_atOffset = 2.5;
    private double m_scoringX = 23.762 - m_atOffset;
    private double m_colWidth = 1.8333;
    private Position2D m_Tag1 = new Position2D(m_scoringX, -9.63, 0);
    private Position2D m_Tag2 = new Position2D(m_scoringX, -4.13, 0);
    private Position2D m_Tag3 = new Position2D(m_scoringX, 1.37, 0);
    // private Arm m_arm;
    // private Claw m_claw;

    private Position2D[] m_scoringColumns = {
            new Position2D(m_scoringX, -9.63 - m_colWidth, 0), // column 1
            new Position2D(m_scoringX, -9.63, 0), // column 2, centered with apriltag 1
            new Position2D(m_scoringX, -9.63 + m_colWidth, 0), // column 3
            new Position2D(m_scoringX, -4.13 - m_colWidth, 0), // column 4
            new Position2D(m_scoringX, -4.13, 0), // column 5, centered with apriltag 2
            new Position2D(m_scoringX, -4.13 + m_colWidth, 0), // column 6
            new Position2D(m_scoringX, 1.37 - m_colWidth, 0), // column 7
            new Position2D(m_scoringX, 1.37, 0), // column 8, centered with apriltag 3
            new Position2D(m_scoringX, 1.37 + m_colWidth, 0) // column 9
    };

	public SequentialAutoCommand(Drivetrain drivetrain, Arm arm, Claw claw, Kinematics kinematics, Targeting targeting,
			StartPositions startPosition) {
		m_drivetrain = drivetrain;
		// m_arm = arm;
		// m_claw = claw;
		m_kinematics = kinematics;
		m_startPosition = startPosition;
        m_targeting = targeting;

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
				// driving
				new DriveTo(new Position2D(6, 0, Math.toRadians(0)), 2.0d, false, m_kinematics, m_drivetrain));

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
