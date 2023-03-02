// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static java.lang.Math.PI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.GyroMoment.WrappedGyro.GyroType;
import frc.robot.swerve.ModuleConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public record RobotSetup(String name, String canivore_name, GyroType gyroType) {}
  // change in robotcontainer
  public static final RobotSetup chris = new RobotSetup("chris", "CAN_Network", GyroType.PIGEON);
  public static final RobotSetup hana = new RobotSetup("hana", "rio", GyroType.NAVX);
  public static final RobotSetup calliope =
      new RobotSetup("calliope", "CAN_Network", GyroType.NAVX);
  public static final RobotSetup spring = new RobotSetup("spring", "rio", GyroType.PIGEON);

  public static final class DrivetrainConstants {

    public static final boolean ADD_TO_DASHBOARD = true;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.508; // 0.62 calliope ,0.508 hana
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.508;

    // Pick the longest side of the robot for this and measure outside bumper to outside bumper
    public static final double ROBOT_LENGTH_WIDTH = 0.698;

    // public static final String CANIVORE_NAME = "rio";

    public static final int PIGEON_ID = 30;

    /**
     * The maximum velocity of the robot in meters per second.
     *
     * <p>This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0
            / 60.0
            * // was 6380.0 / 60.0
            ModuleConfiguration.MK4II_L3.getDriveReduction() // MK4II_L3 for chris
            * ModuleConfiguration.MK4II_L3.getWheelDiameter()
            * PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     *
     * <p>This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a
    // measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        (DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(
                -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(
                -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double DRIVE_kS = 0.6716;
    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double DRIVE_kV = 2.5913;
    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double DRIVE_kA = 0.19321;

    public static final double STEER_kP = 0.2;
    public static final double STEER_kI = 0.0;
    public static final double STEER_kD = 0.1;

    public static final double DRIVE_kP = 0.02;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;
  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 6.0;
    public static final double Y_RATE_LIMIT = 6.0;
    public static final double ROTATION_RATE_LIMIT = 5.0 * PI;

    public static final double HEADING_MAX_VELOCITY = PI * 2;
    public static final double HEADING_MAX_ACCELERATION = PI * 2;

    public static final double HEADING_kP = 2.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.0;

    public static final double HEADING_TOLERANCE = degreesToRadians(1.5);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
        new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
        new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
        new TrapezoidProfile.Constraints(8, 8);

    public static final ProfiledPIDController xController =
        new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    public static final ProfiledPIDController yController =
        new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    public static final ProfiledPIDController omegaController =
        new ProfiledPIDController(1.5, 0, 0, OMEGA_CONSTRAINTS);
  }

  public static class VisionConstants {

    /** Physical location of the camera on the robot, relative to the center of the robot. */
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.4, 0.2, 0.0), new Rotation3d());

    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
  }

  public static class AutoConstants {
    public static TrapezoidProfile.Constraints THETA_CONSTRAINTS =
        new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static double THETA_kP = 1.2; // 1.2 original
    public static double THETA_kI = 0.0;
    public static double THETA_kD = 0.0;

    public static double X_kP = 1.2; // 1.2 original
    public static double X_kI = 0.0;
    public static double X_kD = 0.0;

    public static double Y_kP = 1.2; // 1.2 original
    public static double Y_kI = 0.0;
    public static double Y_kD = 0.0;

    public static PIDController m_translationController =
        new PIDController(
            Constants.AutoConstants.X_kP,
            Constants.AutoConstants.X_kI,
            Constants.AutoConstants.X_kD);
    public static PIDController m_strafeController =
        new PIDController(
            Constants.AutoConstants.Y_kP,
            Constants.AutoConstants.Y_kI,
            Constants.AutoConstants.Y_kD);
    public static PIDController m_thetaController =
        new PIDController(
            Constants.AutoConstants.THETA_kP,
            Constants.AutoConstants.THETA_kI,
            Constants.AutoConstants.THETA_kD);
  }

  public static final class LimeLightConstants {
    public static final double MAX_LIMELIGHT_ERROR_DEGREES =
        1; // limelight max degrees off, max degrees error
    public static final double CAMERA_HEIGHT = 87;
    public static final double TARGET_HEIGHT = 61;
    public static final double MOUNTING_ANGLE = -19;
    public static final double LOWER_DISTANCE_SHOOT = 84;
    public static final double UPPER_DISTANCE_SHOOT = 42;
    public static final double BOTTOM_DISTANCE_SHOOT = 100;
  }

  public static final class Arm {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
    public static final int FALCON_TICKS = 2048;
    public static final int GEAR_RATIO = 64;
    public static final int PWM_CHANNEL_ENCODER_1 = 7;
    public static final int PWM_CHANNEL_ENCODER_2 = 8;
  }
}
