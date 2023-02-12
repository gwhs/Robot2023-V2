// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AutoConstants.THETA_CONSTRAINTS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.DriveTrainConstants;
import frc.robot.GyroMoment.WrappedGyro;
import frc.robot.GyroMoment.WrappedGyro.GyroType;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSpeedController;
import frc.robot.swerve.SwerveSteerController;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.stream.IntStream;

public class DrivetrainSubsystem extends SubsystemBase {

  // private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
  // private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final WrappedGyro gyro = new WrappedGyro(GyroType.PIGEON);
  private final SwerveModule[] swerveModules;
  private final PIDController thetaControllerPID =
      new PIDController(-AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD);
  private final DriveTrainConstants driveTrain;

  private ChassisSpeeds desiredChassisSpeeds;

  public DrivetrainSubsystem(String robotName) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    gyro.configMountPoseRoll(0);
    gyro.configMountPoseYaw(0);

    ShuffleboardLayout frontLeftLayout = null;
    ShuffleboardLayout frontRightLayout = null;
    ShuffleboardLayout backLeftLayout = null;
    ShuffleboardLayout backRightLayout = null;

    if (robotName.equals("hana")) {
      driveTrain = DriveTrainConstants.hana;
    } else if (robotName.equals("spring")) {
      driveTrain = DriveTrainConstants.spring;
    } else if (robotName.equals("chris")) {
      driveTrain = DriveTrainConstants.chris;
    } else {
      driveTrain = DriveTrainConstants.calliope;
    }

    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      frontLeftLayout =
          tab.getLayout("Front Left Module", BuiltInLayouts.kList)
              .withSize(2, 4)
              .withPosition(0, 0);

      frontRightLayout =
          tab.getLayout("Front Right Module", BuiltInLayouts.kList)
              .withSize(2, 4)
              .withPosition(2, 0);

      backLeftLayout =
          tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);

      backRightLayout =
          tab.getLayout("Back Right Module", BuiltInLayouts.kList)
              .withSize(2, 4)
              .withPosition(6, 0);
    }

    // change method we add hana, right now only does spring.
    if (robotName.equals("spring")) {
      swerveModules =
          swerveModuleSpring(frontLeftLayout, frontRightLayout, backLeftLayout, backRightLayout);
    } else if (robotName.equals("hana")) {
      swerveModules =
          swerveModuleHana(frontLeftLayout, frontRightLayout, backLeftLayout, backRightLayout);
    } else if (robotName.equals("chris")) {
      swerveModules =
          swerveModuleChris(frontLeftLayout, frontRightLayout, backLeftLayout, backRightLayout);
    } else {
      swerveModules =
          swerveModuleCalliope(frontLeftLayout, frontRightLayout, backLeftLayout, backRightLayout);
    }

    // Put the motors in brake mode when enabled, coast mode when disabled
    new Trigger(RobotState::isEnabled)
        .onTrue(
            new StartEndCommand(
                () -> {
                  for (SwerveModule swerveModule : swerveModules) {
                    swerveModule.setNeutralMode(NeutralMode.Brake);
                  }
                },
                () -> {
                  for (SwerveModule swerveModule : swerveModules) {
                    swerveModule.setNeutralMode(NeutralMode.Coast);
                  }
                }));
  }

  private SwerveModule[] swerveModuleSpring(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout) {
    /*
     * Specific to the springtrap drivetrain(just the offset)
     */
    DriveTrainConstants driveTrain = DriveTrainConstants.spring;
    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET)
        };
    return swerveModules;
  }

  private SwerveModule[] swerveModuleChris(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout) {
    /*
     * Specific to the springtrap drivetrain(just the offset)
     */
    DriveTrainConstants driveTrain = DriveTrainConstants.chris;
    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET)
        };
    return swerveModules;
  }

  private SwerveModule[] swerveModuleHana(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout) {
    /*
     * Specific to the hana drivetrain(just the offset)
     */
    DriveTrainConstants driveTrain = DriveTrainConstants.hana;
    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET)
        };

    return swerveModules;
  }

  private SwerveModule[] swerveModuleCalliope(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout) {
    /*
     * Specific to the calliope drivetrain(just the offset)
     */
    DriveTrainConstants driveTrain = DriveTrainConstants.calliope;
    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET)
        };
    return swerveModules;
  }

  /**
   * Creates a server module instance
   *
   * @param container shuffleboard layout, or null
   * @param moduleConfiguration module configuration
   * @param driveMotorPort drive motor CAN ID
   * @param steerMotorPort steer motor CAN ID
   * @param steerEncoderPort steer encoder CAN ID
   * @param steerOffset offset for steer encoder
   * @return new swerve module instance
   */
  private static SwerveModule createSwerveModule(
      ShuffleboardLayout container,
      ModuleConfiguration moduleConfiguration,
      int driveMotorPort,
      int steerMotorPort,
      int steerEncoderPort,
      double steerOffset) {

    return new SwerveModule(
        new SwerveSpeedController(driveMotorPort, moduleConfiguration, container),
        new SwerveSteerController(
            steerMotorPort, steerEncoderPort, steerOffset, container, moduleConfiguration));
  }

  public Rotation2d getGyroscopeRotation() {
    return gyro.getRotation2d();
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
    // the angle increase.
    // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public WrappedGyro getGyro() {
    return gyro;
  }

  public void setGyroscopeRotation(double angleDeg) {
    gyro.setYaw(angleDeg);
  }

  public void resetGyro() {
    setGyroscopeRotation(0);
  }

  /**
   * Sets the desired chassis speeds
   *
   * @param chassisSpeeds desired chassis speeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    desiredChassisSpeeds = chassisSpeeds;
  }

  /** Sets the desired speeds to zero */
  public void stop() {
    drive(new ChassisSpeeds());
  }

  /**
   * Gets the actual chassis speeds
   *
   * @return actual chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {
    // Set the swerve module states
    if (desiredChassisSpeeds != null) {
      var desiredStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
      if (desiredChassisSpeeds.vxMetersPerSecond == 0.0
          && desiredChassisSpeeds.vyMetersPerSecond == 0.0
          && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
        var currentStates = getModuleStates();
        // Keep the wheels at their current angle when stopped, don't snap back to straight
        IntStream.range(0, currentStates.length)
            .forEach(i -> desiredStates[i].angle = currentStates[i].angle);
      }

      setModuleStates(desiredStates);
    }
    // Always reset desiredChassisSpeeds to null to prevent latching to the last state (aka motor
    // safety)!!
    desiredChassisSpeeds = null;
  }

  /**
   * Gets the current drivetrain state (velocity, and angle), as reported by the modules themselves.
   *
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveModules)
        .map(module -> module.getState())
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   *
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModules)
        .map(module -> module.getPosition())
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Sets the states of the modules.
   *
   * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    IntStream.range(0, swerveModules.length)
        .forEach(i -> swerveModules[i].setDesiredState(states[i]));
  }

  /**
   * Reseeds the Talon FX steer motors from their CANCoder absolute position. Workaround for "dead
   * wheel"
   */
  public void reseedSteerMotorOffsets() {
    Arrays.stream(swerveModules).forEach(SwerveModule::reseedSteerMotorOffset);
  }
  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   *
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    var thetaController =
        new ProfiledPIDController(
            -AutoConstants.THETA_kP,
            AutoConstants.THETA_kI,
            AutoConstants.THETA_kD,
            THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            poseSupplier,
            DrivetrainConstants.KINEMATICS,
            new PIDController(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
            new PIDController(AutoConstants.Y_kP, AutoConstants.Y_kI, AutoConstants.Y_kD),
            thetaController,
            this::setModuleStates);

    return swerveControllerCommand;
  }

  public PIDController getThetaController() {
    return thetaControllerPID;
  }

  public static PPSwerveControllerCommand followTrajectory(
      DrivetrainSubsystem d, PoseEstimatorSubsystem s, PathPlannerTrajectory traj) {
    return new PPSwerveControllerCommand(
        traj,
        s::getCurrentPose,
        Constants.DrivetrainConstants.KINEMATICS,
        Constants.AutoConstants.m_translationController,
        Constants.AutoConstants.m_strafeController,
        Constants.AutoConstants.m_thetaController,
        d::setModuleStates);
  }
}
