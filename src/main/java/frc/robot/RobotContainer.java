// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotSetup;
import frc.robot.auto.PPSwerveFollower;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.Lime.AfterPPID;
import frc.robot.commands.Lime.PPIDAutoAim;
import frc.robot.commands.Lime.Rotate;
import frc.robot.commands.Lime.Sideways;
import frc.robot.commands.Lime.ToPole;
import frc.robot.commands.ShuffleBoardBen;
import frc.robot.pathfind.MapCreator;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // for ctrl+shift+f, hana, chris, calliope, spring
  // change robot name
  // change this to change robot -----------------v
  // change the same in Robot.java
  private final RobotSetup robot = Constants.hana;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controllertwo = new CommandXboxController(1);
  // Set IP to 10.57.12.11
  // Set RoboRio to 10.57.12.2
  private final LimeLightSub limeLightSub = new LimeLightSub("limelight");

  // Arm
  private final MagicMotion mainArm = new MagicMotion(21, robot.canivore_name());
  private final BoreEncoder shaftEncoder = new BoreEncoder();

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(robot);
  private final PoseEstimatorSubsystem poseEstimator =
      new PoseEstimatorSubsystem(drivetrainSubsystem);
  private final PPIDAutoAim autoAimLime =
      new PPIDAutoAim(drivetrainSubsystem, poseEstimator, limeLightSub);

  private final Rotate rotate = new Rotate(drivetrainSubsystem, poseEstimator, 0);
  private final Sideways sideways = new Sideways(drivetrainSubsystem, limeLightSub);
  private final ToPole toPole = new ToPole(drivetrainSubsystem, limeLightSub);
  private final AfterPPID afterPPID =
      new AfterPPID(drivetrainSubsystem, poseEstimator, limeLightSub);

  private final AutoBalance autoBalance = new AutoBalance(drivetrainSubsystem);
  // Arm

  final List<Obstacle> standardObstacles = FieldConstants.standardObstacles;
  final List<Obstacle> cablePath = FieldConstants.cablePath;
  // final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  // final List<Obstacle> obstacles = FieldConstants.obstacles;

  // VisGraph AStarMap = new VisGraph();
  // final Node finalNode = new Node(4, 4, Rotation2d.fromDegrees(180));
  public MapCreator map = new MapCreator();
  public VisGraph standardMap = new VisGraph();
  public VisGraph cableMap = new VisGraph();

  // LEDStrips
  public final LEDSubsystem m_led = new LEDSubsystem();

  HashMap<String, Command> eventMap = new HashMap<>();

  private final FieldHeadingDriveCommand fieldHeadingDriveCommand =
      new FieldHeadingDriveCommand(
          drivetrainSubsystem,
          () -> poseEstimator.getCurrentPose().getRotation(),
          () ->
              -modifyAxis(controller.getLeftY())
                  * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * drivetrainAmplificationScale(),
          () ->
              -modifyAxis(controller.getLeftX())
                  * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * drivetrainAmplificationScale(),
          () -> -controller.getRightY(),
          () -> -controller.getRightX());

  private final ShuffleBoardBen angleBenCommand =
      new ShuffleBoardBen(drivetrainSubsystem); // add a button

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Sets the color on robot start
    m_led.rainbow();

    Logger logger = Logger.getInstance();
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> poseEstimator.getCurrentPose().getRotation(),
            () ->
                -modifyAxis(controller.getLeftY())
                    * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                    * drivetrainAmplificationScale(),
            () ->
                -modifyAxis(controller.getLeftX())
                    * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                    * drivetrainAmplificationScale(),
            () ->
                -modifyAxis(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis())
                    * drivetrainAmplificationScaleRotation()
                    * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                    / 2));

    drivetrainSubsystem.reseedSteerMotorOffsets();
    // Configure the button bindings

    configureButtonBindings();
    // configureArmBindings();
    // configureLimelightBindings();
    // configureAutoBalanceBindings();
    configureDashboard();
    mainArm.robotInit();
    shaftEncoder.reset();
    setupPathChooser();
  }

  private GenericEntry maxSpeedAdjustment;
  private GenericEntry maxRotationSpeedAdjustment;

  private void configureDashboard() {
    maxSpeedAdjustment =
        Shuffleboard.getTab("Drive")
            .add("Max Speed", 0.2)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();
    maxRotationSpeedAdjustment =
        Shuffleboard.getTab("Drive")
            .add("Max Rotation Speed", 0.2)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();
  }

  private double drivetrainAmplificationScale() {
    // This function multiplies the controller input to reduce the maximum speed,
    // 1 = full speed forward, 0.5 is half speed.
    return maxSpeedAdjustment.getDouble(0.2);
  }

  private double drivetrainAmplificationScaleRotation() {
    // This fun ction multiplies the controller input to reduce the maximum speed,
    // 1 = full speed, 0.5 = speed
    return maxRotationSpeedAdjustment.getDouble(0.2);
  }

  public void startAndBackButton() {
    // Start button reseeds the steer motors to fix dead wheel
    controller
        .start()
        .onTrue(
            Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));

    // Back button resets the robot pose
    controller
        .back()
        .onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Start button reseeds the steer motors to fix dead wheel
    this.startAndBackButton();

    controllertwo
        .start()
        .onTrue(
            Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));

    // Back button resets the robot pose

    // Auto aim
    // Revert - LED
    // controller.b().onTrue(new ChangePipeline(limeLightSub));
    // rotate
    controller.leftBumper().onTrue(sideways);
    // rotate
    controller.rightBumper().onTrue(rotate);

    controllertwo
        .back()
        .onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));

    controller
        // Place mid
        .x // button
        ()
        .onTrue(angleBenCommand); // add a button
    // place low
    // Change back - LED
    // controller.a().toggleOnTrue(fieldHeadingDriveCommand);

    // controller.x().toggleOnTrue(toPole);

    controller.leftStick().toggleOnTrue(fieldHeadingDriveCommand);

    controllertwo
        .x // button
        ()
        .onTrue(angleBenCommand); // add a button
    // place low
    controllertwo.a().toggleOnTrue(fieldHeadingDriveCommand);

    // controller.x().toggleOnTrue(toPole);

    controllertwo.leftStick().toggleOnTrue(fieldHeadingDriveCommand);

    // controller
    // .a()
    // .onTrue(Commands.runOnce(() -> poseEstimator.initializeGyro(0),
    // drivetrainSubsystem));

    // controller
    // .y()
    // .whileTrue(
    // new WPIAStar(
    // drivetrainSubsystem,
    // poseEstimator,
    // new TrajectoryConfig(2, 2),
    // finalNode,
    // obstacles,
    // AStarMap));

    // controller.x().whileTrue(new DriveWithPathPlanner(drivetrainSubsystem,
    // poseEstimator, new PathConstraints(2, 2),
    // new PathPoint(new Translation2d(2.33, 2.03),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270)),
    // new PathPoint(new Translation2d(3, 3),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(180)),
    // new PathPoint(new Translation2d(2, 2),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270)),
    // new PathPoint(new Translation2d(Units.inchesToMeters(200), 2.03),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270))));
    // controller.x().
    // whileTrue(new PPAStar(drivetrainSubsystem, poseEstimator,
    // new PathConstraints(2, 2), finalNode, obstacles, AStarMap));

    controller
        // Place high
        .y()
        .onTrue(
            Commands.sequence(
                new MagicMotionPos(mainArm, 210, 0, 0),
                Commands.waitSeconds(.5),
                new MagicMotionPos(mainArm, 0, 0, 0),
                Commands.waitSeconds(.5),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder)));

    controllertwo
        .y()
        .onTrue(
            Commands.sequence(
                new MagicMotionPos(mainArm, 210, 0, 0),
                Commands.waitSeconds(.5),
                new MagicMotionPos(mainArm, 0, 0, 0),
                Commands.waitSeconds(.5),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder)));

    // change LEDStrip colors

    controller.a().onTrue(Commands.runOnce(() -> m_led.yellow(), m_led));

    controller.b().onTrue(Commands.runOnce(() -> m_led.purple(), m_led));
  }

  private void configureLimelightBindings() {
    this.startAndBackButton();
    controller.x().onTrue(sideways);
    controller.y().onTrue(sideways);
    controller.a().onTrue(sideways);
    controller.b().onTrue(sideways);
  }

  private void configureAutoBalanceBindings() {
    this.startAndBackButton();
    controller.x().onTrue(sideways);
    controller.y().onTrue(sideways);
    controller.a().onTrue(sideways);
    controller.b().onTrue(sideways);
  }

  private void configureArmBindings() {
    this.startAndBackButton();
    controller.x().onTrue(sideways);
    controller.y().onTrue(sideways);
    controller.a().onTrue(sideways);
    controller.b().onTrue(sideways);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  SendableChooser<String> m_chooser = new SendableChooser<>();

  private void setupPathChooser() {
    final ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    m_chooser.setDefaultOption("Straight No Rotation", "StraightNoRotation");
    m_chooser.addOption("Straight With Rotation", "StraightWithRotation");
    m_chooser.addOption("FUN", "FUN");

    tab.add(m_chooser);
  }

  public Command getAutonomousCommand() {
    // return new TestAutonomous(drivetrainSubsystem, poseEstimator);
    return new PPSwerveFollower(
        drivetrainSubsystem,
        poseEstimator,
        m_chooser.getSelected(),
        new PathConstraints(2, 1),
        true);

    // return Commands.print("Starting Command " + m_chooser.getSelected());
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
