// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PPIDAutoAim extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values = {0, 0, 0};
  private boolean sidewaysDone = false;
  private boolean angleDone = false;
  private int noTargets = 0;
  private double distanceError;
  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints positionConstraints =
      new Constraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50,
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50);

  // pid for angle
  private double angleP = 1;
  private double angleI = 0;
  private double angleD = 0;
  private ProfiledPIDController anglePid =
      new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);
  private double targetDistance = 0;
  private double positionP = .01;

  /** Creates a new PPIDAutoAim. */
  public PPIDAutoAim(
      DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      LimeLightSub limeLightSub,
      double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.targetDistance = targetDistance;

    addRequirements(limeLight, drivetrainSubsystem, poseEstimatorSubsystem);
    // addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDone = false;
    sidewaysDone = false;
    // calculates how far it is from target
    distanceError = limeLight.getXDistance() - targetDistance;

    // configuring rotation pid
    anglePid.reset(Math.toRadians(limeLight.getAngle()));
    anglePid.setGoal(Math.toRadians(0));
    anglePid.setTolerance(Math.toRadians(1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (limeLight.hasTarget()) {
      // calculates drive values, pid.calculate called in this function
      noTargets = 0;
      values = chassisValuesLower();
      // makes it drive!
      drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
    } else {
      noTargets++;
    }
    // atgoal and setpoint do not work, so we just brute force it.
    if (Math.abs(limeLight.getAngle()) < .5) {
      angleDone = true;
    } else {
      // sets it to false if position not there yet
      angleDone = false;
    }
    if (Math.abs(distanceError) < 2) {
      sidewaysDone = true;
    } else {
      // sets to false if angle not there yet
      sidewaysDone = false;
    }

    if (noTargets >= 10) {
      sidewaysDone = true;
      angleDone = true;
      System.out.println("target is gone!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(
        "Distance error: " + distanceError + "Dist: " + sidewaysDone + "angle: " + angleDone);
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleDone && sidewaysDone;
  }

  public double[] chassisValuesLower() {
    /*
    [1,2,3]
    1 is x velocity
    2 is y velocity
    3 is degrees rotation
    get the angle using atan2, it returns radians
    use sin and cos to get values to reach max speed
    not really sure about the angle yet.
    */
    distanceError = limeLight.getXDistance() - targetDistance;
    double[] x = new double[3];

    if (!isFinished()) {
      double d = (positionP) * distanceError;
      x[0] = d;
      x[1] = 0;
      x[2] = anglePid.calculate(limeLight.getAngle());
      System.out.println(distanceError + "velo" + d);
    } else {
      x[0] = 0;
      x[1] = 0;
      x[2] = 0;
    }
    return x;
  }
}
