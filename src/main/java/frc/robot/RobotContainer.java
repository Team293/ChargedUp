// RobotBuilder Version: 4.0
// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.MoveClaw;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.SpikeBoard;
import frc.robot.commands.AdjustArm;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.CalibrateExtender;
import frc.robot.commands.CalibratePivot;
import frc.robot.commands.BumpDrive;
import frc.robot.commands.ForzaDrive;
import frc.robot.commands.SequentialAutoCommand;
import frc.robot.commands.TrackTarget;
import frc.robot.commands.RCFDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArm.Node;
import frc.robot.commands.SequentialAutoCommand.StartPositions;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

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
  private static SpikeBoard m_autoBoard;
  public final Kinematics m_kinematics = new Kinematics(new Position2D(0.0, 0.0, 0.0));
  public final Targeting m_targeting = new Targeting();
  public final Drivetrain m_drivetrain = new Drivetrain(m_kinematics);
  public final Arm m_arm = new Arm();
  public final Claw m_claw = new Claw();

  // Joysticks
  public final XboxController m_driverXboxController = new XboxController(0);
  public final XboxController m_operatorXboxController = new XboxController(1);

  public final SendableChooser<Command> m_driveChooser = new SendableChooser<>();
  public final SendableChooser<StartPositions> m_autoChooser = new SendableChooser<>();

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

  public static SpikeBoard getAutoBoard() {
    if (m_autoBoard == null) {
      m_autoBoard = new SpikeBoard("Auto");
    }
    return m_autoBoard;
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
    /******** Operator Controls ********/
    // Invalidate the extender calibration
    final JoystickButton xboxCalibrateExtenderBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kRightBumper.value);
    xboxCalibrateExtenderBtn.whileTrue(new CalibrateExtender(m_arm));

    // Invalidate the pivot calibration
    final JoystickButton xboxCalibratePivotBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kLeftBumper.value);
    xboxCalibratePivotBtn.whileTrue(new CalibratePivot(m_arm));

    // Set arm preset to high location
    final JoystickButton xboxYBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kY.value);
    xboxYBtn.onTrue(new MoveArm(m_arm, Node.HIGH));

    // Set arm preset to mid location
    final JoystickButton xboxXBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kX.value);
    xboxXBtn.onTrue(new MoveArm(m_arm, Node.MID));

    // Set arm preset to hybrid location
    final JoystickButton xboxABtn = new JoystickButton(m_driverXboxController,
        XboxController.Button.kA.value);
    xboxABtn.whileTrue(new AutoAlign(new Limelight(new LimelightHelpers(), m_kinematics), m_kinematics, m_drivetrain));

    // Set arm preset to substation location
    final JoystickButton xboxBBtn = new JoystickButton(m_operatorXboxController,
        XboxController.Button.kB.value);
    xboxBBtn.onTrue(new MoveArm(m_arm, Node.SUBSTATION));

    /******** Driver Controls ********/

    // Bump drive right slightly
    final JoystickButton xboxBumpRight = new JoystickButton(m_driverXboxController,
        XboxController.Button.kRightBumper.value);
    xboxBumpRight.whileTrue(new BumpDrive(m_drivetrain, 0.1d));

    // Bump drive left slightly
    final JoystickButton xboxBumpLeft = new JoystickButton(m_driverXboxController,
        XboxController.Button.kLeftBumper.value);
    xboxBumpLeft.whileTrue(new BumpDrive(m_drivetrain, -0.1d));

    // Set the arm preset to the stow location, inside the robot
    final JoystickButton xboxStowButton = new JoystickButton(m_driverXboxController,
        XboxController.Button.kX.value);
    xboxStowButton.onTrue(new MoveArm(m_arm, Node.STOW));

    // Trigger autobalance
    // final JoystickButton xboxAButton = new JoystickButton(m_driverXboxController,
    // XboxController.Button.kA.value);
    // xboxAButton.onTrue(new AutoBalance(m_drivetrain));

    // Trigger autoalign
    final JoystickButton xboxYButton = new JoystickButton(m_driverXboxController,
        XboxController.Button.kY.value);
    xboxYButton.onTrue(new TrackTarget(m_drivetrain, m_targeting));

    // Added options to the dropdown for driveChooser and putting it into
    // smartdashboard
    m_driveChooser.setDefaultOption("Forza Drive", new ForzaDrive(m_drivetrain, m_driverXboxController));
    m_driveChooser.addOption("Arcade Drive", new ArcadeDrive(m_drivetrain, m_driverXboxController));
    m_driveChooser.addOption("RCF Drive", new RCFDrive(m_drivetrain, m_driverXboxController));
    SmartDashboard.putData(m_driveChooser);

    m_autoChooser.setDefaultOption("Don't Move", SequentialAutoCommand.StartPositions.DONT_MOVE);
    m_autoChooser.addOption("Drive Backward", SequentialAutoCommand.StartPositions.DRIVE_BACKWARD);
    m_autoChooser.addOption("Left Side Score", SequentialAutoCommand.StartPositions.LEFT_SIDE_SCORE);
    
    m_autoChooser.addOption("Score and Engage", SequentialAutoCommand.StartPositions.SCORE_AND_ENGAGE);
    m_autoChooser.addOption("Center Engage", SequentialAutoCommand.StartPositions.CENTER_ENGAGE);
    m_autoChooser.addOption("Right Side Score", SequentialAutoCommand.StartPositions.RIGHT_SIDE_SCORE);
    m_autoChooser.addOption("Score Don't Move", StartPositions.SCORE_DONT_MOVE);
    SmartDashboard.putData(m_autoChooser);
    RobotContainer.getAutoBoard().getTab().add(m_autoChooser).withPosition(0, 0);
    RobotContainer.getAutoBoard().setDouble("first speed", -0.23);
    RobotContainer.getAutoBoard().setDouble("first distance", 6.0);
    RobotContainer.getAutoBoard().setDouble("second speed", -0.085);
    RobotContainer.getAutoBoard().setDouble("second distance", 5);
    RobotContainer.getAutoBoard().setDouble("third speed", 0.325);
    RobotContainer.getAutoBoard().setDouble("third distance", -2.8);
  }

  private Command getDriveCommand() {
    return m_driveChooser.getSelected();
  }

  public void setNeutralMode(NeutralMode nm) {
    m_drivetrain.setNeutralMode(nm);
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
    // Alliance allianceColor = DriverStation.getAlliance();

    // StartPositions startingPosition = StartPositions.INVALID;
    // int location = DriverStation.getLocation();

    // if (allianceColor == Alliance.Blue) {
    // if (1 == location) {
    // startingPosition = StartPositions.BLUE_LEFT;
    // } else if (2 == location) {
    // startingPosition = StartPositions.BLUE_MIDDLE;
    // } else if (3 == location) {
    // startingPosition = StartPositions.BLUE_RIGHT;
    // }
    // } else if (allianceColor == Alliance.Red) {
    // if (1 == location) {
    // startingPosition = StartPositions.RED_LEFT;
    // } else if (2 == location) {
    // startingPosition = StartPositions.RED_MIDDLE;
    // } else if (3 == location) {
    // startingPosition = StartPositions.RED_RIGHT;
    // }
    // } else {
    // System.out.println("WARNING - Invalid alliance color! [" + allianceColor +
    // "]");
    // }

    // if (StartPositions.INVALID == startingPosition) {
    // System.out.println("WARNING - Invalid starting position! [" +
    // startingPosition + "]");
    // } else {
    autoCommand = new SequentialAutoCommand(m_drivetrain, m_arm, m_claw, m_kinematics, m_targeting,
        m_autoChooser.getSelected());
    // }

    return autoCommand;
  }
}
