package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;

public class SequentialAutoCommand extends SequentialCommandGroup {
    public enum StartPositions {
        /**
         * The robot will not move during autonomous. This is the default auto and will
         * earn 0 points.
         */
        DONT_MOVE,
        /**
         * The robot will drive backwards for 10 feet. This has the potential to earn 3
         * points, for mobility.
         */
        DRIVE_BACKWARD,
        /**
         * The robot should start on the left side of the field, based on driver POV.
         * The robot should face the scoring grid and align with the apriltag and should
         * be preloaded with a cube. The robot will lift the arm, score a cube, and then
         * leave the community. This auto can consistently earn 9 points.
         */
        LEFT_SIDE_SCORE,
        /**
         * The robot should start on the right side of the field, based on driver POV.
         * The robot should face the scoring grid and align with the apriltag and should
         * be preloaded with a cube. The robot will lift the arm, score a cube, and then
         * leave the community. This auto can consistently earn 9 points.
         */
        RIGHT_SIDE_SCORE,
        /**
         * The robot should start a foot in front of the scoring grid. The robot should
         * face the scoring grid and align with the apriltag and should be preloaded
         * with a cube. This auto can consistently earn 6 points.
         */
        SCORE_DONT_MOVE,
        /**
         * The robot should start a foot in front of the center scoring grid. The robot
         * should face the scoring grid and align with the apriltag and should be
         * preloaded with a cube. The robot will lift the arm, drive forward, score a
         * cube, and then balance on the charge station. This auto can consistently earn
         * 18 points.
         */
        SCORE_AND_ENGAGE
    }

    private StartPositions m_startPosition;
    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private Arm m_arm;
    private Claw m_claw;

    /**
     * Schedules the autonomous commands based on the starting position of the
     * robot.
     * 
     * @param drivetrain    The drivetrain subsystem
     * @param arm           The arm subsystem
     * @param claw          The claw subsystem
     * @param kinematics    The kinematics subsystem
     * @param targeting     The targeting subsystem
     * @param startPosition The starting position of the robot
     */
    public SequentialAutoCommand(Drivetrain drivetrain, Arm arm, Claw claw, Kinematics kinematics, Targeting targeting,
            StartPositions startPosition) {
        m_drivetrain = drivetrain;
        m_arm = arm;
        m_claw = claw;
        m_kinematics = kinematics;
        m_startPosition = startPosition;

        RobotContainer.getAutoBoard().setBoolean("AutoDone", false);

        switch (m_startPosition) { // Changes the robot path based on the starting position of the robot
            case SCORE_DONT_MOVE:
                // FACE SCORING GRID
                resetKinematics();
                score();
                addCommands(
                        // Drive backwards
                        new DriveTo(new Position2D(-0.5, 0, Math.toRadians(0)), -1.5d, m_kinematics, m_drivetrain),
                        // Lower arm
                        new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES));
                break;

            case LEFT_SIDE_SCORE:
                // FACE SCORING GRID
                resetKinematics();
                score();
                addCommands(
                        // Drive backwards
                        new DriveTo(new Position2D(-0.5, -0.5, Math.toRadians(30)), -1.5d, m_kinematics, m_drivetrain),
                        // Drive backwards and lower arm
                        new ParallelCommandGroup(
                                new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES),
                                new DriveTo(new Position2D(-10, -2, Math.toRadians(0)), -5.0d, m_kinematics,
                                        m_drivetrain)));
                break;

            case SCORE_AND_ENGAGE:
                // FACE SCORING GRID
                resetKinematics();
                score();
                addCommands(
                        // Drive backwards
                        new DriveTo(new Position2D(-0.5, 0, Math.toRadians(0)), -2.0d, m_kinematics, m_drivetrain),
                        // Lower arm
                        new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES),
                        // Drive backwards (~3 feet)
                        new DriveTo(new Position2D(-6, 0, Math.toRadians(0)), -3.5d, m_kinematics, m_drivetrain),
                        // Autobalance
                        new AutoBalance(m_drivetrain));
                break;

            case RIGHT_SIDE_SCORE:
                // FACE SCORING GRID
                resetKinematics();
                score();
                addCommands(
                        // Drive backwards
                        new DriveTo(new Position2D(-0.5, 0.5, Math.toRadians(-30)), -1.5d, m_kinematics, m_drivetrain),
                        // Drive backwards and lower arm
                        new ParallelCommandGroup(
                                new SetArm(m_arm, Arm.STOW_ANGLE, Arm.STOW_R_INCHES),
                                new DriveTo(new Position2D(-10, 2, Math.toRadians(0)), -5.0d, m_kinematics,
                                        m_drivetrain)));
                break;

            case DRIVE_BACKWARD:
                // FACE SCORING GRID
                resetKinematics();
                addCommands(
                        // Drive backwards
                        new DriveTo(new Position2D(-10, 0, Math.toRadians(0)), -5.0d, m_kinematics, m_drivetrain));
                break;

            case DONT_MOVE:
                // Do nothing and don't throw an error
                break;

            default:
                resetKinematics();
                System.out.println("ERROR: Invalid autonomous starting position! [" + m_startPosition + "]");
                break;
        }

        // Alert smart dashboard that autonomous is done
        RobotContainer.getAutoBoard().setBoolean("AutoDone", true);
    }

    // Score a piece in the high node
    public void score() {
        addCommands(
                new SetClawForTime(m_claw, 1.0d, 1.0d),
                // Close claw
                new SetClaw(m_claw, -1.0d, 10.0d),
                // Raise arm
                new SetArm(m_arm, MoveArm.HIGH_ANGLE, Arm.STOW_R_INCHES),
                new Wait(0.5d),
                // Extend arm
                new SetArm(m_arm, MoveArm.HIGH_ANGLE, MoveArm.HIGH_R_INCHES),

                new DriveTo(new Position2D(2, 0, Math.toRadians(0)), 2.0d, m_kinematics, m_drivetrain),
                // Open claw
                new SetClawForTime(m_claw, 1.0d, 10.0d),
                new Wait(1.5d),
                // Retract arm
                new SetArm(m_arm, MoveArm.HIGH_ANGLE, Arm.STOW_R_INCHES));
    }

    private void resetKinematics() {
        addCommands(
                // Reset kinematics to the blue left position
                new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics));
    }
}
