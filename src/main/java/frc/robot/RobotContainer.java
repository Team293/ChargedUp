// RobotBuilder Version: 4.0
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.MoveClaw;
import frc.robot.Constants.AutonomousCommandConstants.StartPositions;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.commands.AdjustArm;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ForzaDrive;
import frc.robot.commands.SequentialAutoCommand;
import frc.robot.commands.TrackTarget;
import frc.robot.commands.ZeroArm;
import frc.robot.commands.RCFDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArm.Node;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.WriteToCSV;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  public final WriteToCSV m_logger = new WriteToCSV();
  public final Arm m_arm = new Arm();
  public final Claw m_claw = new Claw();

  // Joysticks
  public final XboxController m_driverXboxController = new XboxController(0);
  public final XboxController m_operatorXboxController = new XboxController(1);

  public final SendableChooser<Command> m_driveChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Setting default command for drivetrain as VelocityDrive
    m_drivetrain.setDefaultCommand(new ForzaDrive(m_drivetrain, m_driverXboxController));
    m_arm.setDefaultCommand(new AdjustArm(m_arm, m_operatorXboxController));
    m_claw.setDefaultCommand(new MoveClaw(m_claw, m_operatorXboxController));
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
    final JoystickButton xboxTargetBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kLeftBumper.value);
    xboxTargetBtn.whileTrue(new TrackTarget(m_drivetrain, m_targeting));

    final JoystickButton xboxRotate180Btn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kRightBumper.value);
    xboxRotate180Btn.onTrue(new ZeroArm(m_arm));

    final JoystickButton xboxBBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kY.value);
    xboxBBtn.onTrue(new MoveArm(m_arm, m_operatorXboxController, Node.HIGH));

    final JoystickButton xboxXBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kX.value);
    xboxXBtn.onTrue(new MoveArm(m_arm, m_operatorXboxController, Node.MID));

    final JoystickButton xboxABtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kA.value);
    xboxABtn.onTrue(new MoveArm(m_arm, m_operatorXboxController, Node.HYBRID));

    final JoystickButton xboxYBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kB.value);
    xboxYBtn.onTrue(new MoveArm(m_arm, m_operatorXboxController, Node.SUBSTATION));

    // Added options to the dropdown for driveChooser and putting it into
    // smartdashboard
    m_driveChooser.setDefaultOption("Forza Drive", new ForzaDrive(m_drivetrain, m_driverXboxController));
    m_driveChooser.addOption("Arcade Drive", new ArcadeDrive(m_drivetrain, m_driverXboxController));
    m_driveChooser.addOption("RCF Drive", new RCFDrive(m_drivetrain, m_driverXboxController));
    SmartDashboard.putData(m_driveChooser);
  }

  private Command getDriveCommand() {
    return m_driveChooser.getSelected();
  }

  public void setDefaultDrive() {
    m_drivetrain.setDefaultCommand(getDriveCommand());
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
      autoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, m_targeting,
          startingPosition, m_logger);
    }

    return autoCommand;
  }
}
