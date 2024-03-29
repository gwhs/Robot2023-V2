// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CubeLightConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.RobotSetup;
import frc.robot.commands.Arm.ArmSequenceTop;
import frc.robot.commands.Arm.ClawEncoderMoveDown;
import frc.robot.commands.Arm.ClawEncoderMoveUp;
import frc.robot.commands.Arm.ClawOpenClose;
import frc.robot.commands.Arm.GrabPiece;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.commands.Arm.MagicMotionPosShuffleboard;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DynamicDefaultDriveCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.PlaceCone.ChangePipeline;
import frc.robot.commands.PlaceCone.CubePPIDAutoAim;
import frc.robot.commands.PlaceCone.PPIDAutoAim;
import frc.robot.commands.PlaceCone.PlaceHigh;
import frc.robot.commands.PlaceCone.PlaceLow;
import frc.robot.commands.PlaceCone.PlaceMid;
import frc.robot.commands.PlaceCone.Rotate;
import frc.robot.commands.PlaceCone.Sideways;
import frc.robot.commands.PlaceCone.StraightWheel;
import frc.robot.commands.PlaceCone.rotatesideways;
import frc.robot.commands.PlaceCone.toZero;
import frc.robot.commands.autonomous.TestAutoCommands;
import frc.robot.pathfind.MapCreator;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;
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
  private final RobotSetup robot = Constants.ryker;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // Set IP to 10.57.12.11
  // Set RoboRio to 10.57.12.2
  private final LimeLightSub limeLightSub = new LimeLightSub("limelight");

  // Arm
  private final MagicMotion mainArm = new MagicMotion(21, robot.canivore_name());
  private final Claw clawPivot = new Claw(30, MotorType.kBrushless, false);
  private final Claw clawOpenClose = new Claw(31, MotorType.kBrushless, true);
  private final BoreEncoder shaftEncoder = new BoreEncoder(0, 1, "Arm"); // Blue 7 ; Yellow 8
  private final BoreEncoder clawEncoder = new BoreEncoder(2, 3, "Claw");
  private SendableChooser<String> m_chooser;
  private final ArmSequenceTop ArmSequenceCommand =
      new ArmSequenceTop(mainArm, shaftEncoder, clawEncoder, clawPivot);

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

  // VisGraph AStarMap = new VisGraph();
  // final Node finalNode = new Node(4, 4, Rotation2d.fromDegrees(180));
  public MapCreator map = new MapCreator();
  public VisGraph standardMap = new VisGraph();
  public VisGraph cableMap = new VisGraph();

  // mode cone = 0, cube =1
  public int mode = 0;

  // LEDStrips
  public final LEDSubsystem m_led = new LEDSubsystem();

  HashMap<String, Command> eventMap = new HashMap<>();

  private final FieldHeadingDriveCommand fieldHeadingDriveCommand =
      new FieldHeadingDriveCommand(
          drivetrainSubsystem,
          () -> poseEstimator.getCurrentPose().getRotation(),
          () ->
              -modifyAxis(driver.getLeftY())
                  * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * drivetrainAmplificationScale(),
          () ->
              -modifyAxis(driver.getLeftX())
                  * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * drivetrainAmplificationScale(),
          () -> -driver.getRightY(),
          () -> -driver.getRightX());

  private final DynamicDefaultDriveCommand dynamicDefaultDriveCommand =
      new DynamicDefaultDriveCommand(
          drivetrainSubsystem,
          () -> poseEstimator.getCurrentPose().getRotation(),
          () ->
              -modifyAxis(driver.getLeftY())
                  * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * drivetrainDynamicAmplificationScale(),
          () ->
              -modifyAxis(driver.getLeftX())
                  * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                  * drivetrainDynamicAmplificationScale(),
          () ->
              -modifyAxis(driver.getLeftTriggerAxis() - driver.getRightTriggerAxis())
                  * drivetrainDynamicAmplificationScaleRotation()
                  * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                  / 2,
          true);

  // private final ShuffleBoardBen angleBenCommand =
  // new ShuffleBoardBen(drivetrainSubsystem); // add a button

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Logger logger = Logger.getInstance();
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(dynamicDefaultDriveCommand);

    drivetrainSubsystem.reseedSteerMotorOffsets();
    // Configure the button bindings

    // configureButtonBindings();
    // configureArmBindings();
    // configureLimelightBindings();
    // configureAutoBalanceBindings();
    configureDashboard();
    mainArm.robotInit();
    officialBindings();
    // configureArmBindings();

    setupPathChooser();
  }

  private GenericEntry maxSpeedAdjustment;
  private GenericEntry maxRotationSpeedAdjustment;

  // true = speed
  // false = prescision
  private boolean speedState = true;

  private void toggleSpeedState() {
    speedState = !speedState;
  }

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

  private double drivetrainDynamicAmplificationScale() {
    // This function multiplies the controller input to reduce the maximum speed,
    // 1 = full speed forward, 0.5 is half speed.
    if (speedState) {
      return 0.8;
    } else {
      return 0.25;
    }
  }

  private double drivetrainDynamicAmplificationScaleRotation() {
    // This fun ction multiplies the controller input to reduce the maximum speed,
    // 1 = full speed, 0.5 = speed
    if (speedState) {
      return 0.8;
    } else {
      return 0.25;
    }
  }

  public void startAndBackButton() {
    // Start button reseeds the steer motors to fix dead wheel
    driver
        .start()
        .onTrue(
            Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));

    // Back button resets the robot pose
    driver.back().onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));
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

    operator
        .start()
        .onTrue(
            Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));

    // Back button resets the robot pose

    // Auto aim

    // driver.b().onTrue(new ChangePipeline(limeLightSub));
    // rotate
    driver.leftBumper().onTrue(rotate);
    // rotate

    operator
        .back()
        .onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));

    driver
        // Place mid
        .x // button
        ()
        .onTrue(sideways); // add a button
    // place low

    // driver.a().toggleOnTrue(fieldHeadingDriveCommand);

    driver.x().onTrue(new ChangePipeline(limeLightSub));
    // driver.b().onTrue(rotate);
    // driver.x().onTrue(new ChangePipeline(limeLightSub));
    // driver.b().onTrue(rotate);
    // driver.y().onTrue(autoAimLime1);

    operator
        .x // button
        ()
        .onTrue(new ChangePipeline(limeLightSub)); // add a button
    // place low
    operator.a().toggleOnTrue(fieldHeadingDriveCommand);

    driver.x().onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));

    operator.leftStick().toggleOnTrue(fieldHeadingDriveCommand);
    driver
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
    driver
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
    driver
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

    // driver
    // .y()
    // .whileTrue(
    // new WPIAStar(
    // drivetrainSubsystem,
    // poseEstimator,
    // new TrajectoryConfig(2, 2),
    // finalNode,
    // obstacles,
    // AStarMap));

    // driver.x().whileTrue(new DriveWithPathPlanner(drivetrainSubsystem,
    // poseEstimator, new PathConstraints(2, 2),
    // new PathPoint(new Translation2d(2.33, 2.03),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270)),
    // new PathPoint(new Translation2d(3, 3),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(180)),
    // new PathPoint(new Translation2d(2, 2),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270)),
    // new PathPoint(new Translation2d(Units.inchesToMeters(200), 2.03),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270))));
    // driver.x().
    // whileTrue(new PPAStar(drivetrainSubsystem, poseEstimator,
    // new PathConstraints(2, 2), finalNode, obstacles, AStarMap));

    // driver.y().onTrue(straightWheel1);
    // operator
    // // Place high //5 , 2.5, 5
    // .y()
    // .onTrue(
    // Commands.sequence(
    // Commands.print("START"),
    // // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder,
    // "Cube").withTimeout(1.5),
    // // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
    // // Commands.waitSeconds(.25),
    // // new MagicMotionPos(mainArm, 40, 1, 1, 5),
    // new MagicMotionPosShuffleboard(mainArm, 175, 2.75, 5),
    // // Commands.waitSeconds(.1),
    // // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
    // // Commands.waitSeconds(),
    // new MagicMotionPos(mainArm, 0, 3, 1.5, .5),
    // Commands.waitSeconds(.5),
    // // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
    // // Commands.waitSeconds(.3),
    // new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));

    // // CUBE
    // operator
    // .rightBumper()
    // .onTrue(
    // Commands.either(
    // new ClawEncoderMoveDown(-125, clawPivot, clawEncoder,
    // "Cube").withTimeout(1.5),
    // Commands.sequence(
    // Commands.print("Encoder Pos" + -clawEncoder.getRaw() / 8192. * 360.),
    // Commands.parallel(
    // new ClawOpenCloseShuffleBoard(75, 5, clawOpenClose),
    // Commands.waitSeconds(1)),
    // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
    // new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
    // clawEncoder::posDown));

    operator
        .rightBumper()
        .onTrue(new GrabPiece(mainArm, shaftEncoder, clawPivot, clawEncoder, clawOpenClose, mode));

    // CONE
    operator
        // Place high //5 , 2.5, 5
        .y()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder,
                // "Cube").withTimeout(1.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // Commands.waitSeconds(.25),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                new MagicMotionPosShuffleboard(mainArm, 190, 2.75, 5, shaftEncoder),
                // Commands.waitSeconds(.1),
                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 30, 3, 1.5, .5),
                Commands.waitSeconds(.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),

                // new MagicMotionPosShuffleboard(mainArm, 190, 2.75, 5),
                // new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                // Commands.waitSeconds(.5),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));
    // CUBE
    // operator
    // .rightBumper()
    // .onTrue(
    // Commands.either(
    // new ClawEncoderMoveDown(-125, clawPivot, clawEncoder,
    // "Cube").withTimeout(1.5),
    // Commands.sequence(
    // Commands.print("Encoder Pos" + -clawEncoder.getRaw() / 8192. * 360.),
    // Commands.parallel(
    // new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose),
    // Commands.waitSeconds(1)),
    // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
    // new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
    // clawEncoder::posDown));

    // CONE
    operator
        .leftTrigger()
        .onTrue(
            Commands.either(
                new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "CONE").withTimeout(3),
                Commands.sequence(
                    Commands.parallel(
                        new ClawOpenClose(100, 20, clawOpenClose), Commands.waitSeconds(1)),
                    new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CONE").withTimeout(3),
                    new ClawOpenClose(20, 20, clawOpenClose)),
                clawEncoder::posDown));
  }

  private void configureLimelightBindings() {
    this.startAndBackButton();
    // driver.a().onTrue(Commands.runOnce(() -> m_led.toggleLED()));
    driver.leftBumper().onTrue(rotate);
    driver.rightBumper().onTrue(new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 80)); //

    driver.a().onTrue(new ChangePipeline(limeLightSub));
    driver.x().onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));
    driver
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
    driver
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
  }

  private void configureArmBindings() {
    this.startAndBackButton();
    driver.x().onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));
    driver
        .x()
        .onTrue(
            Commands.either(
                new CubePPIDAutoAim(
                    drivetrainSubsystem, limeLightSub, CubeLightConstants.MID_DISTANCE_SHOOT),
                new PPIDAutoAim(
                    drivetrainSubsystem, limeLightSub, LimeLightConstants.MID_DISTANCE_SHOOT),
                limeLightSub::checkPipe));
    driver
        .a()
        .onTrue(
            Commands.sequence(
                new PPIDAutoAim(
                    drivetrainSubsystem, limeLightSub, LimeLightConstants.MID_DISTANCE_SHOOT)));
    driver
        .b()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(0.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                // for cube throw 100deg, 10vel, 10 accel
                // FOR CUBE PLACE, 210, 2.75 VELO, 3.5 ACCEL
                new MagicMotionPosShuffleboard(mainArm, 197, 2.75, 5.5, shaftEncoder),

                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 20, 3, 1.5, .5),
                // Commands.waitSeconds(.25),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));
    // INTAKE PICK-UP CONE
    driver.rightBumper().onTrue(new ChangePipeline(limeLightSub));
  }

  private void setupPathChooser() {
    final ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    m_chooser = new SendableChooser<>();
    tab.add(m_chooser);
    m_chooser.addOption("D or F Place and engage", "D||F1E");
    m_chooser.addOption("E place and engage", "E1E");
    m_chooser.addOption("Place one CONE from any node", "PlaceOne");
    m_chooser.addOption("A or I Place and Mobility", "AIMobile");
    m_chooser.addOption("B or H Place and Mobility", "BHMobile");
    m_chooser.addOption("I 1 Mobility and engage", "HajelPath");
    m_chooser.addOption("A 1 Mobility and engage", "APlaceME");
    m_chooser.addOption("C place and engage", "C1+E");
    m_chooser.addOption("G place and engage", "G1+E");
    m_chooser.addOption("straight with rotation", "StraightWithRotation");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TestAutoCommands vendingMachine =
        new TestAutoCommands(
            drivetrainSubsystem,
            poseEstimator,
            mainArm,
            shaftEncoder,
            m_chooser.getSelected(),
            limeLightSub,
            clawEncoder,
            clawPivot,
            clawOpenClose);

    return new ParallelCommandGroup(
            vendingMachine.getAutoCommand().withTimeout(14.499999), Commands.waitSeconds(14.5))
        .andThen(new StraightWheel(drivetrainSubsystem, true));

    // return vendingMachine.getAutoCommand();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, Constants.TeleopDriveConstants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void officialBindings() {
    // Start button reseeds the steer motors to fix dead wheel
    this.startAndBackButton();
    driver
        .start()
        .onTrue(
            Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));
    driver.back().onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));

    // all need binding
    // Cone

    driver.b().onTrue(ArmSequenceCommand.starting());

    driver
        .leftBumper()
        .onTrue(
            new rotatesideways(drivetrainSubsystem, poseEstimator, limeLightSub).withTimeout(2));
    // Cube Toss
    driver
        .y()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(0.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                // for cube throw 100deg, 10vel, 10 accel
                // FOR CUBE PLACE, 210, 2.75 VELO, 3.5 ACCEL
                new MagicMotionPos(mainArm, 235, 2.75, 3, 1),
                Commands.waitSeconds(.25),
                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 20, 3, 1.5, .5),
                // Commands.waitSeconds(.25),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));
    // driver.y().onTrue(new MagicMotionPosShuffleboard(mainArm, 235, 2.75, 3, clawEncoder));
    driver
        .y()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(0.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                // for cube throw 100deg, 10vel, 10 accel
                // FOR CUBE PLACE, 210, 2.75 VELO, 3.5 ACCEL
                new MagicMotionPos(mainArm, 235, 2.75, 3, 1),

                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 20, 3, 1.5, .5),
                // Commands.waitSeconds(.25),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));
    driver.rightBumper().onTrue(new toZero(drivetrainSubsystem, poseEstimator));
    driver
        .x()
        .onTrue(
            Commands.either(
                new CubePPIDAutoAim(
                    drivetrainSubsystem, limeLightSub, CubeLightConstants.MID_DISTANCE_SHOOT),
                new PPIDAutoAim(
                    drivetrainSubsystem, limeLightSub, LimeLightConstants.MID_DISTANCE_SHOOT),
                limeLightSub::checkPipe));

    driver
        .a()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> dynamicDefaultDriveCommand.toggleSlewRate()),
                Commands.runOnce(() -> toggleSpeedState())));
    // driver.start().onTrue(fieldHeadingDriveCommand);
    // driver.back().onTrue(fieldHeadingDriveCommand);
    // operator.x().onTrue(new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 70));

    // needs binding
    // operator
    //     .a()
    //     .onTrue(
    //         );
    operator.rightTrigger().onTrue(new ChangePipeline(limeLightSub));
    operator.x().onTrue(new StraightWheel(drivetrainSubsystem, true));
    operator
        .b()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> mainArm.swapMode(), mainArm),
                Commands.runOnce(() -> System.out.println("Mode Num: " + mainArm.getMode())),
                Commands.runOnce(() -> m_led.toggleLED(), m_led),
                new ChangePipeline(limeLightSub)));
    operator.rightBumper().onTrue(new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5));

    // FREE BUTTONT
    // operator.leftBumper().onTrue();
    operator
        .start()
        .onTrue(
            Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));
    operator
        .back()
        .onTrue(Commands.runOnce(poseEstimator::set180FieldPosition, drivetrainSubsystem));

    operator
        .y()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(0.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                // for cube throw 100deg, 10vel, 10 accel
                // FOR CUBE PLACE, 210, 2.75 VELO, 3.5 ACCEL
                new MagicMotionPos(mainArm, 20, 20, 20, 1),

                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 20, 3, 1.5, .5),
                // Commands.waitSeconds(.25),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));
    // INTAKE PICK-UP CONE
    // driver
    //     .b()
    //     .onTrue(
    //         Commands.either(
    //             new ClawEncoderMoveDown(-155, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //             Commands.sequence(
    //                 Commands.parallel(
    //                     new ClawOpenClose(100, 30, clawOpenClose), Commands.waitSeconds(1)),
    //                 new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //                 new ClawOpenClose(20, 30, clawOpenClose)),
    //             clawEncoder::posDown));

    // INTAKE UP & DOWN
    // operator
    //     .a()
    //     .onTrue(
    //         Commands.either(
    //             new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //             new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //             clawEncoder::posDown2));

    operator
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> dynamicDefaultDriveCommand.toggleSlewRate()),
                Commands.runOnce(() -> toggleSpeedState())));

    operator
        .leftBumper()
        .onTrue(new MagicMotionPosShuffleboard(mainArm, 235, 2.75, 3, clawEncoder));

    operator
        .leftBumper()
        .onTrue(
            Commands.sequence(
                Commands.print("START"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(0.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                // for cube throw 100deg, 10vel, 10 accel
                // FOR CUBE PLACE, 210, 2.75 VELO, 3.5 ACCEL
                new MagicMotionPos(mainArm, 100, 10, 10, 1),
                Commands.waitSeconds(.25),
                // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
                // Commands.waitSeconds(),
                new MagicMotionPos(mainArm, 20, 3, 1.5, .5),
                // Commands.waitSeconds(.25),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                // Commands.waitSeconds(.3),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)));
    // operator
    //     .a()
    //     .onTrue(
    //         Commands.either(
    //             new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //             new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CONE").withTimeout(3),
    //             clawEncoder::posDown2));

  }
  // zoey
  /*
   * a = cone
   * y = ryker cube
   * x = rotate
   * rightbump rotate
   * leftbump = stopeverything
   *
   * ben
   * a, b, y stop everyting
   * x change pipeline
   * start
   */
}
