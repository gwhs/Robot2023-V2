// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import java.util.List;

public class PPIDAutoAim extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values = {0, 0, 0};
  private boolean sidewaysDone = false;
  private boolean angleDone = false;
  private int noTargets = 0;
  private double distanceError;

  private double anglePDefault;
  private double angleIDefault;
  private double angleDDefault;
  private double positionPDefault;

  private GenericEntry anglePEntry;
  private GenericEntry angleIEntry;
  private GenericEntry angleDEntry;
  private GenericEntry positionPEntry;

  private final ShuffleboardTab tab;
  private int times = 0;

  // second param on constraints is estimated, should be max accel, not max speed,
  // but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  // second param on constraints is estimated, should be max accel, not max speed,
  // but lets say it
  // gets there in a second
  private Constraints positionConstraints =
      new Constraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50,
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50);

  // pid for angle
  private double angleP = 1;
  private double angleI = 0;
  private double angleD = .06;
  private ProfiledPIDController anglePid =
      new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);
  private double targetDistance = 0;
  private double positionP = .007;

  /** Creates a new PPIDAutoAim. */
  public PPIDAutoAim(
      DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub, double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.targetDistance = targetDistance;

    anglePDefault = 2;
    angleIDefault = 0;
    angleDDefault = .08;
    positionPDefault = 0.025;

    tab = Shuffleboard.getTab("Drive");

    ShuffleboardLayout PIDConstants =
        tab.getLayout("AutoAim PID Constants", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0);

    if (PIDConstants.getComponents().isEmpty()) {

      anglePEntry =
          PIDConstants.add("Angle P Constant", anglePDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      angleIEntry =
          PIDConstants.add("Angle I Constant", angleIDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      angleDEntry =
          PIDConstants.add("Angle D Constant", angleDDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      positionPEntry =
          PIDConstants.add("Position P Constant", positionPDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();

    } else {
      List<ShuffleboardComponent<?>> widgets = PIDConstants.getComponents();
      anglePEntry = ((SimpleWidget) widgets.get(0)).getEntry();
      angleIEntry = ((SimpleWidget) widgets.get(1)).getEntry();
      angleDEntry = ((SimpleWidget) widgets.get(2)).getEntry();
      positionPEntry = ((SimpleWidget) widgets.get(3)).getEntry();
    }

    addRequirements(limeLight, drivetrainSubsystem);
    // addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDone = false;
    sidewaysDone = false;
    // calculates how far it is from target
    distanceError = limeLight.getConeXDistance() - targetDistance;

    angleP = anglePEntry.getDouble(anglePDefault);
    angleI = angleIEntry.getDouble(angleIDefault);
    angleD = angleDEntry.getDouble(angleDDefault);
    positionP = positionPEntry.getDouble(positionPDefault);

    anglePid = new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);

    // configuring rotation pid
    anglePid.reset(Math.toRadians(limeLight.getAngle()));
    anglePid.setGoal(Math.toRadians(-3.7));
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
    if (Math.abs(limeLight.getTx() + 3.7) < 1) {
      angleDone = true;
    } else {
      // sets it to false if position not there yet
      angleDone = false;
    }
    if (Math.abs(distanceError) < 1) {
      sidewaysDone = true;
    } else {
      // sets to false if angle not there yet
      sidewaysDone = false;
    }
    if (sidewaysDone && angleDone) {
      times++;
    } else {
      times = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("done");
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (angleDone && sidewaysDone && times > 5) || noTargets > 5;
  }

  public double[] chassisValuesLower() {
    /*
     * [1,2,3]
     * 1 is x velocity
     * 2 is y velocity
     * 3 is degrees rotation
     * get the angle using atan2, it returns radians
     * use sin and cos to get values to reach max speed
     * not really sure about the angle yet.
     */
    distanceError = limeLight.getConeXDistance() - targetDistance;
    double[] x = new double[3];

    double d = (positionP) * distanceError;
    x[0] = sidewaysDone ? 0 : d;
    x[1] = 0;
    x[2] = angleDone ? 0 : anglePid.calculate(limeLight.getAngle());
    System.out.println(distanceError + "velo" + d);

    return x;
  }
}
