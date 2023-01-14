package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.WriteToCSV;

import static frc.robot.Constants.AutonomousCommandConstants.*;

public class SequentialAutoCommand extends SequentialCommandGroup {
        private StartPositions m_startPosition;
        private Drivetrain m_drivetrain;
        private Kinematics m_kinematics;
        private Feeder m_feeder;
        private Targeting m_targeting;
        private Launcher m_launcher;
        private WriteToCSV m_logger;

    public SequentialAutoCommand(Drivetrain drivetrain, Kinematics kinematics, Feeder feeder, Targeting targeting,
            Launcher launcher, StartPositions startPosition, WriteToCSV logger) {

        m_drivetrain = drivetrain;
        m_kinematics = kinematics;
        m_startPosition = startPosition;
        m_feeder = feeder;
        m_targeting = targeting;
        m_launcher = launcher;
        m_logger = logger;

        SmartDashboard.putBoolean("AutoDone", false);

        switch (m_startPosition) { // Changes the robot path based on the starting position of the robot Left,
                                   // Middle, Right
            case BLUE_LEFT:
                addCommands(
                    // Reset kinematics to the blue left position
                    new ResetKinematics(new Position2D(0, 0, Math.toRadians(90)), m_drivetrain, m_kinematics),

                    // Drive to the first ball and collect it
                    deadline(new DriveTo(new Position2D(0, 6, Math.toRadians(90)), 2.0d, false, m_kinematics, m_drivetrain),
                             new BallControl(m_feeder)),

                    // Turn around to face the hub
                    deadline(new Rotate(m_drivetrain, 180.0), 
                             new BallControl(m_feeder)),

                        // Aim at the hub
                        // Fire both balls!
                    deadline(new Wait(3),
                             new TrackTarget(m_drivetrain, m_targeting, m_launcher)),
                
                    new Fire(m_feeder, m_launcher, m_targeting,  m_logger, AUTO_LAUNCHER_RPM)
                );
                // new ParallelRaceGroup(
                // new Fire(m_feeder, m_launcher, m_targeting),
                // new Wait(3.0)
                // )
                // Turns launcher off
                // new LauncherOff(m_launcher),
                // // Rotate torwards opposing ball
                // new Rotate(m_drivetrain, -90.0),
                // // Drive to the opposing ball and collect it
                // new DriveTo(new Position2D(-7.359, 10.412, Math.toRadians(0)), 2.0d, false,
                // m_kinematics, m_drivetrain),
                // // Rotate away from hub
                // new Rotate(m_drivetrain, 135.0),
                // // Dumps opposing ball
                // new ParallelRaceGroup(
                // new Fire(m_feeder, m_launcher, m_targeting),
                // new Wait(3.0)
                // )
                break;

            case BLUE_MIDDLE:
                addCommands(
                        // Reset kinematics to the blue middle position
                        new ResetKinematics(new Position2D(-5.350, -5.363, Math.toRadians(133.5)), m_drivetrain,
                                m_kinematics),
                        // Drive to the first blue ball and collect it
                        new DriveTo(new Position2D(-10.412, -7.359, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Turn around to face the hub
                        new Rotate(m_drivetrain, 180.0),
                        // Aim at the hub
                        new TrackTarget(m_drivetrain, m_targeting, m_launcher),
                        // Fire both balls!
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)),
                        // Turns launcher off
                        new LauncherOff(m_launcher),
                        // Rotate towards terminal
                        new Rotate(m_drivetrain, 180),
                        // Drive to terminal ball and get ball
                        new DriveTo(new Position2D(-23.507, -9.810, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate towards launching position
                        new Rotate(m_drivetrain, 180),
                        // Drive to launching position
                        new DriveTo(new Position2D(-10.412, -7.359, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Launches terminal ball
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)));
                break;

            case BLUE_RIGHT:
                addCommands(
                        // Reset kinematics to blue right position
                        new ResetKinematics(new Position2D(-1.995, -7.628, Math.toRadians(91.5)), m_drivetrain,
                                m_kinematics),
                        // Drives to first ball and collects
                        new DriveTo(new Position2D(-2.159, -12.566, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotates to face the hub
                        new Rotate(m_drivetrain, 180.0),
                        // Aims at the hub
                        new TrackTarget(m_drivetrain, m_targeting, m_launcher),
                        // Fire both balls!
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)),
                        // Turns launcher off
                        new LauncherOff(m_launcher),
                        // Rotates towards opposing ball
                        new Rotate(m_drivetrain, -90.0),
                        // Drives and pickups opposing ball
                        new DriveTo(new Position2D(2.814, -12.436, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate away from the hub
                        new Rotate(m_drivetrain, 180.0),
                        // Dumps enemy ball
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)));
                break;

            case RED_LEFT:
                addCommands(
                        // Reset kinematics to red left position
                        new ResetKinematics(new Position2D(7.004, -3.621, Math.toRadians(46.5)), m_drivetrain,
                                m_kinematics),
                        // Drive to the first ball and collect it
                        new DriveTo(new Position2D(10.783, -6.804, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotates to face the hub
                        new Rotate(m_drivetrain, 180.0),
                        // Aims at the hub
                        new TrackTarget(m_drivetrain, m_targeting, m_launcher),
                        // Fire both balls!
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)),
                        // Turns launcher off
                        new LauncherOff(m_launcher),
                        // Rotates towards opposing ball
                        new Rotate(m_drivetrain, -90.0),
                        // Drive to opposing ball
                        new DriveTo(new Position2D(7.359, -10.412, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate away from hub
                        new Rotate(m_drivetrain, 135.0),
                        // Dumps opposing ball
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)));
                break;

            case RED_MIDDLE:
                addCommands(
                        // Reset kinematics to red middle position
                        new ResetKinematics(new Position2D(5.350, 5.363, Math.toRadians(-43.5)), m_drivetrain,
                                m_kinematics),
                        // Drive to the first blue ball and collect it
                        new DriveTo(new Position2D(10.412, 7.359, Math.toRadians(-43.5)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate to face the hub
                        new Rotate(m_drivetrain, 180.0),
                        // Aim at the hub
                        new TrackTarget(m_drivetrain, m_targeting, m_launcher),
                        // Fire both balls!
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)),
                        // Turns launcher off
                        new LauncherOff(m_launcher),
                        // Rotate towards terminal
                        new Rotate(m_drivetrain, 180),
                        // Drive to terminal and pickup ball
                        new DriveTo(new Position2D(23.507, 9.810, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate towards hub
                        new Rotate(m_drivetrain, 180),
                        // Drive to launching position
                        new DriveTo(new Position2D(10.412, 7.359, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Launch terminal ball
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)));
                break;

            case RED_RIGHT:
                addCommands(
                        // Reset kinematics to red right position
                        new ResetKinematics(new Position2D(1.995, 7.628, Math.toRadians(-91.5)), m_drivetrain,
                                m_kinematics),
                        // Drives to first ball and collects
                        new DriveTo(new Position2D(2.159, 12.566, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate to face the hub
                        new Rotate(m_drivetrain, 180.0),
                        // Aim at the hub
                        new TrackTarget(m_drivetrain, m_targeting, m_launcher),
                        // Fires both balls!
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)),
                        // Turns launcher off
                        new LauncherOff(m_launcher),
                        // Rotates towards opposing ball
                        new Rotate(m_drivetrain, -90.0),
                        // drives to opposing ball
                        new DriveTo(new Position2D(-2.814, 12.436, Math.toRadians(0)), 2.0d, false, m_kinematics,
                                m_drivetrain),
                        // Rotate away from hub
                        new Rotate(m_drivetrain, 180.0),
                        // Dumps enemy ball
                        new ParallelRaceGroup(
                                new Fire(m_feeder, m_launcher, m_targeting, m_logger),
                                new Wait(3.0)));
                break;

            default:
                System.out.println("ERROR: Invalid autonomous starting position! [" + m_startPosition + "]");
                break;
        }

        // Alert smart dashboard that autonomous is done
        SmartDashboard.putBoolean("AutoDone", true);
    }
}
