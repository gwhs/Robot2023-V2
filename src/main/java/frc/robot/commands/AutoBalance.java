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

  private double currentAngle;
  private double currentDPS;
  private double speed;
  private double pConstant;
  private double dConstant;
  private Timer timer;
  private double error;
  private double tolerance;
  private double desiredEngageTime; // in seconds
  private double maxSpeed;
  private final ShuffleboardTab tab;
  private final GenericEntry pConstantEntry;
  private final GenericEntry dConstantEntry;
  private final GenericEntry toleranceEntry;
  private final GenericEntry maxSpeedEntry;
  private final GenericEntry desiredEngageTimeEntry;

  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
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

    pConstantEntry = input.add("p Constant", 0.008).withWidget(BuiltInWidgets.kTextView).getEntry();
    dConstantEntry = input.add("d Constant", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    toleranceEntry = input.add("Tolerance", 2.5).withWidget(BuiltInWidgets.kTextView).getEntry();
    maxSpeedEntry = input.add("Max Speed", 0.5).withWidget(BuiltInWidgets.kTextView).getEntry();
    desiredEngageTimeEntry =
        input.add("Desired Engage Time", 1).withWidget(BuiltInWidgets.kTextView).getEntry();

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredEngageTime = desiredEngageTimeEntry.getDouble(1); // in seconds
    maxSpeed = maxSpeedEntry.getDouble(0.5);
    pConstant = pConstantEntry.getDouble(0.008);
    dConstant = dConstantEntry.getDouble(0);
    tolerance = toleranceEntry.getDouble(2.5); // in degrees
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = drivetrainSubsystem.getRoll();
    currentDPS = drivetrainSubsystem.getRollRate();

    error = currentAngle - 0;

    speed = error * pConstant + currentDPS * dConstant;

    if (speed > maxSpeed) {
      speed = maxSpeed;
    } else {

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
