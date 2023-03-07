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

public class Rotate extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PoseEstimatorSubsystem poseEstimatorSubsystem;
  private LimeLightSub limeLight;
  private double degrees;
  private double[] values = {0, 0, 0};
  private boolean angleDone = false;
  private boolean sideDone = false;
  private double p = .02;
  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  //// second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second

  private double angleP = .01;
  private double angleI = 0;
  private double angleD = 0;

  private ProfiledPIDController anglePid =
      new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);

  /** Creates a new Rotate. */
  public Rotate(
      DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      LimeLightSub limeLightSub,
      double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limeLight = limeLightSub;
    this.degrees = degrees;

    addRequirements(poseEstimatorSubsystem, drivetrainSubsystem);
    // addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDone = false;
    sideDone = false;

    // configure rotation pid
    System.out.println(poseEstimatorSubsystem.getAngle());
    anglePid.reset(poseEstimatorSubsystem.getAngle());
    anglePid.setGoal(180);
    anglePid.setTolerance(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // gets drive values to align to 0 degrees, field oriented.
    values = chassisValuesLower();
    drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
    // System.out.printf(
    //     "X equals %.2f PID moves %.2f%n", poseEstimatorSubsystem.getAngle(), values[2]);
    // setpoint and atgoal don't work, just brute forced.
    if (Math.abs(180 - poseEstimatorSubsystem.getAngle()) < .5) {
      angleDone = true;
    } else {
      angleDone = false;
    }
    if (Math.abs(limeLight.getTx()) < .5) {
      sideDone = true;
    } else {
      sideDone = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Rotated done");
    drivetrainSubsystem.drive(new ChassisSpeeds(.001, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleDone;
  }

  public double[] chassisValuesLower() {
    /*
    [1,2,3]
    1 is x velocity
    2 is y velocity
    3 is degrees rotation
    */

    double[] x = new double[3];

    x[0] = 0.00001;
    x[1] = Math.abs(limeLight.getTx()) > 1 ? (-p * limeLight.getTx()) : 0;
    x[2] =
        angleDone
            ? 0
            : -((poseEstimatorSubsystem.getAngle() % 180)
                    - Math.copySign(180, poseEstimatorSubsystem.getAngle()))
                * angleP;
    // System.out.println(
    //     "sideways speed"
    //         + x[1]
    //         + "\nangle speed"
    //         + x[2]
    //         + "\nangle"
    //         + poseEstimatorSubsystem.getAngle());
    System.out.println("Angle: " + poseEstimatorSubsystem.getAngle());
    return x;
  }
}
