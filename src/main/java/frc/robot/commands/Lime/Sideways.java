// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lime;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class Sideways extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values = {0, 0, 0};
  private boolean sidewaysDone = false;
  private boolean angleDone = false;
  private double angleGoal = 0;
  // private final GenericEntry pentry;
  // private final GenericEntry dentry;
  // private final GenericEntry ientry;
  private double initAngle;
  private double distanceError;
  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  //// second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints positionConstraints =
      new Constraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50,
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50);

  private double angleP = 5;
  private double angleI = 0;
  private double angleD = 0;
  // private final ShuffleboardTab tab;
  private ProfiledPIDController anglePid =
      new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);

  private double positionP = .025;
  private double positionI = 0;
  private double positionD = 0;
  private ProfiledPIDController positionPid =
      new ProfiledPIDController(positionP, positionI, positionD, positionConstraints);

  /** Creates a new AutoAimLime. */
  public Sideways(DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDone = false;
    sidewaysDone = false;
    distanceError = limeLight.getXDistance() - LimeLightConstants.LOWER_DISTANCE_SHOOT;

    // rotating to align
    anglePid.reset(Math.toRadians(limeLight.getTx()));
    anglePid.setGoal(Math.toRadians(0));
    anglePid.setTolerance(Math.toRadians(1));

    // moving to align
    positionPid.reset(limeLight.hasTarget() ? distanceError : 0);
    positionPid.setGoal(0);
    positionPid.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // add pids
    values = chassisValuesLower();
    if (limeLight.hasTarget()) {
      drivetrainSubsystem.drive(new ChassisSpeeds(.01, values[1], .001));
    }
    // System.out.printf(
    // "X equals %.2f PID moves %.2f%n", poseEstimatorSubsystem.getAngle(), values[2]);
    // atgoal is not working, it needs it to be == setpoint and be in setpoint.
    // setpoint just makes sure it's in the tolerance, doesn't work
    if (Math.abs(limeLight.getAngle()) < 1) {
      angleDone = true;
    } else {
      angleDone = false;
    }
    // System.out.println(distanceError);
    if (Math.abs(distanceError) < 2) {
      sidewaysDone = true;
    } else {
      sidewaysDone = false;
    }
    // System.out.printf("angle done? %s distance %s %n", angleDone, sidewaysDone);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Final Horiz-error", limeLight.getTx());
    SmartDashboard.putNumber("Final Vert-error", limeLight.getTy());
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // && angleDone
    return Math.abs(limeLight.getTx()) < 1;
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
    double[] x = new double[3];

    // System.out.printf(distanceError + "d" + d);

    x[0] = 0;

    x[1] = positionPid.calculate(limeLight.getTx());
    // calculate is overloaded, second parameter is angle goal if it changes
    x[2] = 0;

    // getAngle()
    //     / (((Math.sqrt(x[0] * x[0]) + x[1] * x[1]))
    //         / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    return x;
  }
}
