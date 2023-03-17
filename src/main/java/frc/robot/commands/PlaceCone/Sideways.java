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

public class Sideways extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values = {0, 0, 0};
  private boolean sidewaysDone = false;
  private PoseEstimatorSubsystem poseEstimatorSubsystem;
  private int noTarget = 0;
  private int seen;
  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  //// second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints constraints =
      new Constraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50,
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50);

  private double P = .004;
  private double I = 0;
  private double D = 0;
  private ProfiledPIDController pid = new ProfiledPIDController(P, I, D, constraints);
  private int times = 0;

  /** Creates a new AutoAimLime. */
  public Sideways(
      DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;

    addRequirements(drivetrainSubsystem, limeLightSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sidewaysDone = false;
    // rotating to align
    pid.reset(Math.toRadians(limeLight.getTx()));
    pid.setGoal(Math.toRadians(0));
    pid.setTolerance(Math.toRadians(1));
    times = 0;

    // configure rotation pid
    System.out.println(poseEstimatorSubsystem.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // add pids
    values = chassisValuesLower();
    if (limeLight.hasTarget()) {
      noTarget = 0;
      drivetrainSubsystem.drive(new ChassisSpeeds(.00001, values[1], values[2]));
    } else {
      noTarget++;
    }
    // atgoal is not working, it needs it to be == setpoint and be in setpoint.
    // setpoint just makes sure it's in the tolerance, doesn't work
    if (Math.abs(limeLight.getTx()) < .5) {
      sidewaysDone = true;
      times ++;
    } else {
      sidewaysDone = false;
      times = 0;
    }

    if (noTarget >= 10) {
      sidewaysDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Sideways done");
    drivetrainSubsystem.drive(new ChassisSpeeds(0.00, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sidewaysDone && times > 5;
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

    x[0] = 0;
    x[1] = sidewaysDone ? 0 : pid.calculate(limeLight.getTx());
    x[2] = 0;

    return x;
    // d
  }
}
