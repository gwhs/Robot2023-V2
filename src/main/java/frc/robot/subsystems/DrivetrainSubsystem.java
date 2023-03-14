// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.DriveTrainConstants;
import frc.robot.GyroMoment.WrappedGyro;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSpeedController;
import frc.robot.swerve.SwerveSteerController;
import java.util.Arrays;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {

  // private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
  // private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final WrappedGyro gyro; // hana
  // private final WrappedGyro gyro = new WrappedGyro(GyroType.PIGEON); // chris
  private final SwerveModule[] swerveModules;

  private final DriveTrainConstants driveTrain;

  private ChassisSpeeds desiredChassisSpeeds;

  public DrivetrainSubsystem(Constants.RobotSetup setup) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    gyro = new WrappedGyro(setup.gyroType(), setup.canivore_name());
    gyro.configMountPoseRoll(0);
    gyro.configMountPoseYaw(0);

    ShuffleboardLayout frontLeftLayout = null;
    ShuffleboardLayout frontRightLayout = null;
    ShuffleboardLayout backLeftLayout = null;
    ShuffleboardLayout backRightLayout = null;

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

    if (setup.name().equals("spring")) {
      driveTrain = DriveTrainConstants.spring;
      swerveModules =
          swerveModuleSpring(
              frontLeftLayout,
              frontRightLayout,
              backLeftLayout,
              backRightLayout,
              setup.canivore_name());
    } else if (setup.name().equals("hana")) {
      driveTrain = DriveTrainConstants.hana;
      swerveModules =
          swerveModuleHana(
              frontLeftLayout,
              frontRightLayout,
              backLeftLayout,
              backRightLayout,
              setup.canivore_name());
    } else if (setup.name().equals("chris")) {
      driveTrain = DriveTrainConstants.chris;
      swerveModules =
          swerveModuleChris(
              frontLeftLayout,
              frontRightLayout,
              backLeftLayout,
              backRightLayout,
              setup.canivore_name());
    } else if (setup.name().equals("chuck")) {
      driveTrain = DriveTrainConstants.chuck;
      swerveModules =
          swerveModuleChuck(
              frontLeftLayout,
              frontRightLayout,
              backLeftLayout,
              backRightLayout,
              setup.canivore_name());
    } else {
      driveTrain = DriveTrainConstants.calliope;
      swerveModules =
          swerveModuleCalliope(
              frontLeftLayout,
              frontRightLayout,
              backLeftLayout,
              backRightLayout,
              setup.canivore_name());
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
      ShuffleboardLayout backRightLayout,
      String canivoreName) {
    /*
     * Specific to the springtrap drivetrain(just the offset)
     */

    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET,
              canivoreName)
        };
    return swerveModules;
  }

  private SwerveModule[] swerveModuleChris(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout,
      String canivoreName) {
    /*
     * Specific to the springtrap drivetrain(just the offset)
     */

    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4II_L3_FL,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4II_L3_FR,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET,
              canivoreName)
        };
    return swerveModules;
  }

  private SwerveModule[] swerveModuleChuck(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout,
      String canivoreName) {
    /*
     * Specific to the springtrap drivetrain(just the offset)
     */

    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4II_L3_FL,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4II_L3_FR,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4II_L3,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET,
              canivoreName)
        };
    return swerveModules;
  }

  private SwerveModule[] swerveModuleHana(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout,
      String canivoreName) {
    /*
     * Specific to the hana drivetrain(just the offset)
     */

    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4_L3,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4_L3,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4_L3,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4_L3,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET,
              canivoreName)
        };

    return swerveModules;
  }

  private SwerveModule[] swerveModuleCalliope(
      ShuffleboardLayout frontLeftLayout,
      ShuffleboardLayout frontRightLayout,
      ShuffleboardLayout backLeftLayout,
      ShuffleboardLayout backRightLayout,
      String canivoreName) {
    /*
     * Specific to the calliope drivetrain(just the offset)
     */

    SwerveModule[] swerveModules =
        new SwerveModule[] {
          createSwerveModule(
              frontLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              frontRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backLeftLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
              driveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
              driveTrain.BACK_LEFT_MODULE_STEER_OFFSET,
              canivoreName),
          createSwerveModule(
              backRightLayout,
              ModuleConfiguration.MK4I_L2,
              driveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
              driveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
              driveTrain.BACK_RIGHT_MODULE_STEER_OFFSET,
              canivoreName)
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
      double steerOffset,
      String canivoreName) {

    return new SwerveModule(
        new SwerveSpeedController(driveMotorPort, moduleConfiguration, container, canivoreName),
        new SwerveSteerController(
            steerMotorPort,
            steerEncoderPort,
            steerOffset,
            container,
            moduleConfiguration,
            canivoreName));
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
      Logger.getInstance().recordOutput("DriveTrainSub/DesireStates", desiredStates);
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
    states[0].speedMetersPerSecond = states[0].speedMetersPerSecond * 2;
    states[1].speedMetersPerSecond = states[1].speedMetersPerSecond * 1 / 2;
    states[2].speedMetersPerSecond = states[2].speedMetersPerSecond * 1/6;
    states[3].speedMetersPerSecond = states[3].speedMetersPerSecond * 4;
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
}
