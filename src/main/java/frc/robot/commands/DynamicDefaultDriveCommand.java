package frc.robot.commands;

import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.Y_RATE_LIMIT;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DynamicDefaultDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Rotation2d> robotAngleSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT);

  private boolean useSlewRate;

  /**
   * Constructor
   *
   * @param drivetrainSubsystem drivetrain
   * @param robotAngleSupplier supplier for the current angle of the robot
   * @param translationXSupplier supplier for translation X component, in meters per second
   * @param translationYSupplier supplier for translation Y component, in meters per second
   * @param rotationSupplier supplier for rotation component, in radians per second
   */
  public DynamicDefaultDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Rotation2d> robotAngleSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    this.useSlewRate = true;
    addRequirements(drivetrainSubsystem);
  }

  public DynamicDefaultDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Rotation2d> robotAngleSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      boolean useSlewRate) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    this.useSlewRate = useSlewRate;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    resetSlewRate();
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented
    // movement
    if (useSlewRate) {
      drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
              translateYRateLimiter.calculate(translationYSupplier.getAsDouble()),
              -rotationRateLimiter.calculate(rotationSupplier.getAsDouble()),
              robotAngleSupplier.get()));
    } else {
      drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationXSupplier.getAsDouble(),
              translationYSupplier.getAsDouble(),
              -rotationSupplier.getAsDouble(),
              robotAngleSupplier.get()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  public void toggleSlewRate() {
    this.useSlewRate = !this.useSlewRate;
    resetSlewRate();
  }

  public void resetSlewRate() {
    var robotAngle = robotAngleSupplier.get();

    // Calculate field relative speeds
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    var robotSpeeds =
        new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond * robotAngle.getCos()
                - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin(),
            chassisSpeeds.vyMetersPerSecond * robotAngle.getCos()
                + chassisSpeeds.vxMetersPerSecond * robotAngle.getSin(),
            chassisSpeeds.omegaRadiansPerSecond);

    // Reset the slew rate limiters, in case the robot is already moving
    translateXRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    translateYRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
    rotationRateLimiter.reset(robotSpeeds.omegaRadiansPerSecond);
  }
}
