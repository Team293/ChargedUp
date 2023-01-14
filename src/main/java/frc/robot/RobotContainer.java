// RobotBuilder Version: 4.0
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.Constants.AutonomousCommandConstants.StartPositions;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // Robots Subsystems
  private static RobotContainer m_robotContainer = new RobotContainer();
  public final Kinematics m_kinematics = new Kinematics(new Position2D(0.0, 0.0, 0.0));
  public final Targeting m_targeting = new Targeting();
  public final Drivetrain m_drivetrain = new Drivetrain(m_kinematics);
  public final Launcher m_launcher = new Launcher();
  public final Feeder m_feeder = new Feeder();
  public final Climb m_climb = new Climb();
  public final WriteToCSV m_logger = new WriteToCSV();

  // Joysticks
  public final XboxController m_driverXboxController = new XboxController(0);
  public final XboxController m_operatorXboxController = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // SmartDashboard Buttons
    SmartDashboard.putData("ArcadeDrive", new ArcadeDrive(m_drivetrain, m_driverXboxController));

    // Configure the button bindings
    configureButtonBindings();

    // Setting default command for drivetrain as VelocityDrive
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, m_driverXboxController));
    m_feeder.setDefaultCommand(new BallControl(m_feeder));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    final JoystickButton xboxFeedBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kRightBumper.value);
    xboxFeedBtn.whileHeld(new Fire(m_feeder, m_launcher, m_targeting, m_logger));

    final JoystickButton xboxTargetBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kLeftBumper.value);
    xboxTargetBtn.whileHeld(new TrackTarget(m_drivetrain, m_targeting, m_launcher));

    // Raise the launcher piston
    final POVButton dpadUpButton = new POVButton(m_operatorXboxController, 0);
    dpadUpButton.whenPressed(new ExtendClimb(m_climb));

    // Lower the launcher piston
    final POVButton dpadDownButton = new POVButton(m_operatorXboxController, 180);
    dpadDownButton.whenPressed(new RetractClimb(m_climb));

    final JoystickButton xboxRotate180Btn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kA.value);
    xboxRotate180Btn.whenPressed(new Rotate(m_drivetrain, 180.0));

    final JoystickButton forceDump = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kX.value);
    forceDump.whileHeld(new Fire(m_feeder, m_launcher, m_targeting,  m_logger, 2000.0d));

    final JoystickButton eject = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kB.value);
    eject.whileHeld(new Eject(m_feeder));

    final JoystickButton badBallDump = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kY.value);
    badBallDump.whileHeld(new Fire(m_feeder, m_launcher, m_targeting,  m_logger, 1000.0d)); /////BJM Why do we have this?
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    // When either the alliance colour check or the location check fails it defaults
    // to the blue left side
    Command autoCommand = null;
    Alliance allianceColor = DriverStation.getAlliance();

    StartPositions startingPosition = StartPositions.INVALID;
    int location = 1;

    if (allianceColor == Alliance.Blue) {
      if (1 == location) {
        startingPosition = StartPositions.BLUE_LEFT;
      } else if (2 == location) {
        startingPosition = StartPositions.BLUE_MIDDLE;
      } else if (3 == location) {
        startingPosition = StartPositions.BLUE_RIGHT;
      }
    } else if (allianceColor == Alliance.Red) {
      if (1 == location) {
        startingPosition = StartPositions.BLUE_LEFT;
      } else if (2 == location) {
        startingPosition = StartPositions.RED_MIDDLE;
      } else if (3 == location) {
        startingPosition = StartPositions.RED_RIGHT;
      }
    } else {
      System.out.println("WARNING - Invalid alliance color! [" + allianceColor + "]");
    }

    if (StartPositions.INVALID == startingPosition) {
      System.out.println("WARNING - Invalid starting position! [" + startingPosition + "]");
    } else {
      autoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, m_feeder, m_targeting, m_launcher,
          startingPosition, m_logger);
    }

    return autoCommand;
  }

  public Command getTeleopCommand() {
    return new ArcadeDrive(m_drivetrain, m_driverXboxController);
  }
}
