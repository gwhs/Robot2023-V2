// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private Timer timer;

  private double pConstant;
  private double dConstant;
  private double tolerance;
  private double desiredEngageTime; // in seconds
  private double maxSpeed;
  private double pConstantInitial;

  private double pConstantDefault;
  private double dConstantDefault;
  private double maxSpeedDefault;
  private double desiredEngageTimeDefault;
  private double toleranceDefault;
  private double pConstantInitialDefault;

  private int state;
  private double epsilonRollRate;
  private double epsilonRollRateDefault;
  private final GenericEntry epsilonRollRateEntry;

  private final ShuffleboardTab tab;
  private final GenericEntry pConstantEntry;
  private final GenericEntry dConstantEntry;
  private final GenericEntry toleranceEntry;
  private final GenericEntry maxSpeedEntry;
  private final GenericEntry desiredEngageTimeEntry;
  private final GenericEntry pConstantInitialEntry;

  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;

    pConstantDefault = 0.004;
    dConstantDefault = -0.001;
    toleranceDefault = 2.5;
    maxSpeedDefault = 0.5;
    desiredEngageTimeDefault = 1;
    pConstantInitialDefault = 0.2;
    epsilonRollRateDefault = 2;

    timer = new Timer();
    timer.stop();
    tab = Shuffleboard.getTab("Auto Balance");

    ShuffleboardLayout orientation =
        tab.getLayout("Robot Orientation", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    ShuffleboardLayout input =
        tab.getLayout("Constant Inputs", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);

    orientation.addNumber("Yaw", () -> drivetrainSubsystem.getYaw());
    orientation.addNumber("Pitch", () -> drivetrainSubsystem.getPitch());
    orientation.addNumber("Roll", () -> drivetrainSubsystem.getRoll());
    orientation.addNumber("Yaw Rate", () -> drivetrainSubsystem.getYawRate());
    orientation.addNumber("Pitch Rate", () -> drivetrainSubsystem.getPitchRate());
    orientation.addNumber("Roll Rate", () -> drivetrainSubsystem.getRollRate());

    pConstantEntry =
        input.add("p Constant", pConstantDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
    dConstantEntry =
        input.add("d Constant", dConstantDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
    toleranceEntry =
        input.add("Tolerance", toleranceDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
    maxSpeedEntry =
        input.add("Max Speed", maxSpeedDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
    desiredEngageTimeEntry =
        input
            .add("Desired Engage Time", desiredEngageTimeDefault)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    pConstantInitialEntry =
        input
            .add("Initial p Constant", pConstantInitialDefault)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    epsilonRollRateEntry =
        input
            .add("Epsilon Roll Rate Threshold", epsilonRollRateDefault)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    desiredEngageTime = desiredEngageTimeEntry.getDouble(desiredEngageTimeDefault); // in seconds
    maxSpeed = maxSpeedEntry.getDouble(maxSpeedDefault);
    pConstant = pConstantEntry.getDouble(pConstantDefault);
    dConstant = dConstantEntry.getDouble(dConstantDefault);
    tolerance = toleranceEntry.getDouble(toleranceDefault); // in degrees
    pConstantInitial = pConstantInitialEntry.getDouble(pConstantInitialDefault);
    epsilonRollRate = epsilonRollRateEntry.getDouble(epsilonRollRateDefault);
    System.out.printf(
        "max = %f, p Constant = %f, d Constant = %f, tolerance = %f, Desired Engage Time = %f, Initial p Constant = %f, Epsilon Roll Rate Threshold = %f",
        maxSpeed,
        pConstant,
        dConstant,
        tolerance,
        desiredEngageTime,
        pConstantInitial,
        epsilonRollRate);

    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = drivetrainSubsystem.getRoll();
    double currentDPS = drivetrainSubsystem.getRollRate();

    double error = currentAngle - 0;

    double speed = 0;

    if (Math.abs(currentDPS) >= Math.abs(epsilonRollRate)) {
      state = 1;
    }

    if (state == 0) {
      speed = Math.copySign(pConstantInitial, error);
    }

    if (state == 1) {
      speed = error * pConstant + currentDPS * dConstant;
    }

    if (speed > maxSpeed) {
      speed = maxSpeed;
    } else if (speed < -maxSpeed) {
      speed = -maxSpeed;
    }

    if (Math.abs(error) <= tolerance) {
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }

    drivetrainSubsystem.drive(new ChassisSpeeds(speed, 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(desiredEngageTime);
  }
}
