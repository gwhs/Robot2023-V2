// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lime;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class PPIDAutoAim extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values;
  private boolean horizDone = false;
  private boolean angleDone = false;
  private double angleGoal = 0;
  private final GenericEntry pentry;
  private final GenericEntry dentry;
  private final GenericEntry ientry;
  private double initAngle;
  private double targetY = LimeLightConstants.MAX_LIMELIGHT_ERROR_DEGREES;
  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  private double p = 3;
  private double i = 0;
  private double d = .1;
  private final ShuffleboardTab tab;
  private ProfiledPIDController pid = new ProfiledPIDController(p, i, d, angleConstraints);

  /** Creates a new AutoAimLime. */
  public PPIDAutoAim(DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
    tab = Shuffleboard.getTab("limepid");
    ShuffleboardLayout input =
        tab.getLayout("Constant Inputs", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    pentry = input.add("p Constant", p).withWidget(BuiltInWidgets.kTextView).getEntry();
    dentry = input.add("d Constant", d).withWidget(BuiltInWidgets.kTextView).getEntry();
    ientry = input.add("i constant", i).withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    horizDone = false;
    angleDone = false;

    initAngle = limeLight.hasTarget() ? limeLight.getTx() : 0;
    pid.reset(Math.toRadians(initAngle));
    p = pentry.getDouble(p);
    d = dentry.getDouble(d);
    i = ientry.getDouble(i);
    // goal velocity is 0 (overloaded constructor)
    pid.setGoal(Math.toRadians(angleGoal));
    pid.setTolerance(Math.toRadians(10));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // add pids
    values = chassisValuesLower();
    if (limeLight.hasTarget()) {
      drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
    } else {
      System.out.println("NO TARGET FOUND BY LIMELIGHT");
    }
    if (Math.abs(limeLight.getXDistance() - LimeLightConstants.LOWER_DISTANCE_SHOOT) < 2) {
      horizDone = true;
    } else {
      horizDone = false;
    }
    System.out.printf("X equals %f PID moves %f%n", limeLight.getTx(), values[2]);
    System.out.println();
    //atgoal is not working, it needs it to be == setpoint and be in setpoint. 
    //setpoint just makes sure it's in the tolerance
    if (pid.atSetpoint()) {
      angleDone = true;
    } else {
      angleDone = false;
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
    // && angleDone
    return angleDone;
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

    // motor.set(controller.calculate(encoder.getDistance(), goal));

    double distanceError = limeLight.getXDistance() - LimeLightConstants.LOWER_DISTANCE_SHOOT;
    double[] x = new double[3];
    x[0] = 0;
    x[1] = 0;
    // calculate is overloaded, second parameter is angle goal if it changes
    x[2] = (pid.calculate(Math.toRadians(limeLight.getTx())));

    // getAngle()
    //     / (((Math.sqrt(x[0] * x[0]) + x[1] * x[1]))
    //         / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    return x;
  }
}
