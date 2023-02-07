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
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class toPole extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PoseEstimatorSubsystem poseEstimatorSubsystem;
  private LimeLightSub limeLight;
  private double[] values = {0, 0, 0};
  private boolean done = false;
  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second

  //// second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints Constraints =
      new Constraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 5,
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 5);

  private double P = 5;
  private double I = 0;
  private double D = 0;
  // private final ShuffleboardTab tab;
  private ProfiledPIDController Pid = new ProfiledPIDController(P, I, D, Constraints);

  /** Creates a new AutoAimLime. */
  public toPole(DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;

    // rotating to align
    Pid.reset(limeLight.getXDistance());
    Pid.setGoal(0);
    Pid.setTolerance(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // add pids
    if (limeLight.hasTarget()) {
      values = chassisValuesLower();
      drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
      System.out.printf(
          "X equals %.2f PID moves %.2f%n", poseEstimatorSubsystem.getAngle(), values[2]);
    }

    // atgoal is not working, it needs it to be == setpoint and be in setpoint.
    // setpoint just makes sure it's in the tolerance, doesn't work
    if (Math.abs(limeLight.getTx()) < 2) {
      done = true;
    } else {
      done = false;
    }
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
    return done;
  }

  public double[] chassisValuesLower() {
    /*
    [1,2,3]
    1 is x velocity
    2 is y velocity
    3 is degrees rotation
    */

    // motor.set(controller.calculate(encoder.getDistance(), goal));
    double[] x = new double[3];
    x[0] = Pid.calculate(limeLight.getXDistance());
    ;
    // calculate is overloaded, second parameter is angle goal if it changes
    x[1] = 0;
    x[2] = 0;
    return x;
  }
}
