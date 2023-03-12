// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotSetup;
import frc.robot.commands.Arm.ClawEncoderMoveDown;
import frc.robot.commands.Arm.ClawEncoderMoveUp;
import frc.robot.commands.Arm.ClawOpenClose;
import frc.robot.commands.Arm.ClawOpenCloseShuffleBoard;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.commands.Arm.MagicMotionPosShuffleboard;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.PlaceCone.AllLime;
import frc.robot.commands.PlaceCone.ChangePipeline;
import frc.robot.commands.PlaceCone.PPIDAutoAim;
import frc.robot.commands.PlaceCone.PlaceHigh;
import frc.robot.commands.PlaceCone.PlaceLow;
import frc.robot.commands.PlaceCone.PlaceMid;
import frc.robot.commands.PlaceCone.Rotate;
import frc.robot.commands.PlaceCone.Sideways;
import frc.robot.commands.autonomous.TestAutoCommands;
import frc.robot.pathfind.MapCreator;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
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
  private final RobotSetup robot = Constants.chuck;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controllertwo = new CommandXboxController(1);
  // Set IP to 10.57.12.11
  // Set RoboRio to 10.57.12.2
  private final LimeLightSub limeLightSub = new LimeLightSub("limelight");

  // Arm
  private final MagicMotion mainArm = new MagicMotion(21, robot.canivore_name());
  private final Claw clawPivot = new Claw(30, MotorType.kBrushless, false);
  private final Claw clawOpenClose = new Claw(31, MotorType.kBrushless, true);
  private final BoreEncoder shaftEncoder = new BoreEncoder(0, 1); // Blue 7 ; Yellow 8
  private final BoreEncoder clawEncoder = new BoreEncoder(2, 3);
  private SendableChooser<String> m_chooser;

  // Led status
  private int status = 1;

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(robot);
  private final PoseEstimatorSubsystem poseEstimator =
      new PoseEstimatorSubsystem(drivetrainSubsystem);

  private final Rotate rotate = new Rotate(drivetrainSubsystem, poseEstimator, limeLightSub);
  private final Sideways sideways = new Sideways(drivetrainSubsystem, poseEstimator, limeLightSub);

  private final AutoBalance autoBalance = new AutoBalance(drivetrainSubsystem);
  final List<Obstacle> standardObstacles = FieldConstants.standardObstacles;
  final List<Obstacle> cablePath = FieldConstants.cablePath;
  // final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  // final List<Obstacle> obstacles = FieldConstants.obstacles;
  private final AllLime allLime = new AllLime(drivetrainSubsystem, poseEstimator, limeLightSub);

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

  // private final ShuffleBoardBen angleBenCommand =
  //     new ShuffleBoardBen(drivetrainSubsystem); // add a button

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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

    // configureButtonBindings();
    // configureArmBindings();
    configureLimelightBindings();
    // configureAutoBalanceBindings();
    configureDashboard();
    mainArm.robotInit();

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

    // controller.b().onTrue(new ChangePipeline(limeLightSub));
    // rotate
    controller.leftBumper().onTrue(rotate);
    // rotate

    controller.rightBumper().onTrue(allLime); //

    controllertwo
        .back()
        .onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));

    controller
        // Place mid
        .x // button
        ()
        .onTrue(sideways); // add a button
    // place low

    // controller.a().toggleOnTrue(fieldHeadingDriveCommand);

    controller.x().onTrue(new ChangePipeline(limeLightSub));
    // controller.b().onTrue(rotate);
    // controller.x().onTrue(new ChangePipeline(limeLightSub));
    // controller.b().onTrue(rotate);
    // controller.y().onTrue(autoAimLime1);

    controllertwo
        .x // button
        ()
        .onTrue(new ChangePipeline(limeLightSub)); // add a button
    // place low
    controllertwo.a().toggleOnTrue(fieldHeadingDriveCommand);

    controller
        .x()
        .onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));

    controllertwo.leftStick().toggleOnTrue(fieldHeadingDriveCommand);
    controller
        .b()
        .onTrue(
            new PlaceLow(
                drivetrainSubsystem,
                poseEstimator,
                limeLightSub,
                mainArm,
                shaftEncoder,
                clawEncoder,
                clawPivot,
                220));
    controller
        .y()
        .onTrue(
            new PlaceMid(
                drivetrainSubsystem,
                limeLightSub,
                mainArm,
                shaftEncoder,
                clawEncoder,
                clawPivot,
                220));
    controller
        .rightBumper()
        .onTrue(
            new PlaceHigh(
                drivetrainSubsystem,
                poseEstimator,
                limeLightSub,
                mainArm,
                shaftEncoder,
                clawEncoder,
                clawPivot,
                220));

    // controller212
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

    // controller.y().onTrue(straightWheel1);
    controllertwo
        // Place high //5 , 2.5, 5
        .y()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // Commands.waitSeconds(.25),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                new MagicMotionPosShuffleboard(mainArm, 190, 2.75, 5),
                // Commands.waitSeconds(.1),
                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 30, 3, 1.5, .5),
                Commands.waitSeconds(.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),

                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));

    // CUBE
    controllertwo
        .rightBumper()
        .onTrue(
            Commands.either(
                new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
                Commands.sequence(
                    Commands.print("Encoder Pos" + -clawEncoder.getRaw() / 8192. * 360.),
                    Commands.parallel(
                        new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose),
                        Commands.waitSeconds(1)),
                    new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
                    new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
                clawEncoder::posDown));

    // CONE
    // controllertwo
    //     .leftTrigger()
    //     .onTrue(
    //         Commands.either(
    //             new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //             Commands.sequence(
    //                 Commands.parallel(
    //                     new ClawOpenClose(-85, 20, clawOpenClose), Commands.waitSeconds(1)),
    //                 new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //                 new ClawOpenClose(0, 20, clawOpenClose)),
    //             clawEncoder::posDown));

    controller.a().onTrue(Commands.runOnce(() -> toggleLED()));
  }

  private void configureLimelightBindings() {
    this.startAndBackButton();
    controller.a().onTrue(Commands.runOnce(() -> toggleLED()));
    controller.leftBumper().onTrue(rotate);
    controller.rightBumper().onTrue(new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 80)); //

    controller.a().onTrue(new ChangePipeline(limeLightSub));
    controller
        .x()
        .onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));
    controller
        .y()
        .onTrue(
            Commands.either(
                new PlaceMid(
                    drivetrainSubsystem,
                    limeLightSub,
                    mainArm,
                    shaftEncoder,
                    clawEncoder,
                    clawPivot,
                    220),
                new PlaceMid(
                    drivetrainSubsystem,
                    limeLightSub,
                    mainArm,
                    shaftEncoder,
                    clawEncoder,
                    clawPivot,
                    215),
                limeLightSub::checkPipe));
    controller
        .b()
        .onTrue(
            Commands.either(
                new PlaceHigh(
                    drivetrainSubsystem,
                    poseEstimator,
                    limeLightSub,
                    mainArm,
                    shaftEncoder,
                    clawEncoder,
                    clawPivot,
                    190),
                new PlaceHigh(
                    drivetrainSubsystem,
                    poseEstimator,
                    limeLightSub,
                    mainArm,
                    shaftEncoder,
                    clawEncoder,
                    clawPivot,
                    185),
                limeLightSub::checkPipe));
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

  private void setupPathChooser() {
    final ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    m_chooser = new SendableChooser<>();

    m_chooser.setDefaultOption("Straight No Rotation", "StraightNoRotation");
    m_chooser.addOption("Straight With Rotation", "StraightWithRotation");
    m_chooser.addOption("FUN", "FUN");

    tab.getLayout("Auto Paths", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0)
        .add(m_chooser);
  }

  private void toggleLED() {
    if (m_led.getLedMode() == LEDMode.YELLOW) {
      m_led.setLedMode(LEDMode.PURPLE);
    } else {
      m_led.setLedMode(LEDMode.YELLOW);
    }

    // if (m_led.getLedMode() == LEDMode.YELLOW) {
    //   m_led.setLedMode(LEDMode.PURPLE);
    // } else if (m_led.getLedMode() == LEDMode.PURPLE) {
    //   m_led.setLedMode(LEDMode.EMERGENCY);
    // } else if (m_led.getLedMode() == LEDMode.EMERGENCY) {
    //   m_led.setLedMode(LEDMode.GREEN);
    // } else if (m_led.getLedMode() == LEDMode.GREEN) {
    //   m_led.setLedMode(LEDMode.ORANGE);
    // } else if (m_led.getLedMode() == LEDMode.ORANGE) {
    //   m_led.setLedMode(LEDMode.TEAMCOLOR);
    // } else if (m_led.getLedMode() == LEDMode.TEAMCOLOR) {
    //   m_led.setLedMode(LEDMode.RAINBOW);
    // } else if (m_led.getLedMode() == LEDMode.RAINBOW) {
    //   m_led.setLedMode(LEDMode.PINK);
    // } else {
    //   m_led.setLedMode(LEDMode.YELLOW);
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new TestAutonomous(drivetrainSubsystem, poseEstimator);
    TestAutoCommands vendingMachine =
        new TestAutoCommands(
            drivetrainSubsystem, poseEstimator, mainArm, shaftEncoder, m_chooser.getSelected());

    return vendingMachine.getAutoCommand();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, Constants.TeleopDriveConstants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
