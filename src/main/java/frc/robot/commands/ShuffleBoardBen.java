// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GyroMoment.WrappedGyro;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ShuffleBoardBen extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  // private ShuffleboardTab tab = Shuffleboard.getTab("ShuffleBoardBen");

  private double initialAngle;
  private GenericEntry angleEntry;
  private double angleBen;
  private WrappedGyro gyro;

  // angle bop

  /** Creates a new AutoBalance. */
  public ShuffleBoardBen(DrivetrainSubsystem drivetrainSubsystem) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    gyro = this.drivetrainSubsystem.getGyro();

    final ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    initialAngle = 0.5;

    Shuffleboard.selectTab("Angle Ben");

    ShuffleboardLayout orientation =
        tab.getLayout("Robot Orientation", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    ShuffleboardLayout input =
        tab.getLayout("Constant Inputs", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);

    orientation.addNumber("Yaw", () -> gyro.getYaw()); // gets angle

    angleEntry = input.add("p Angle", initialAngle).withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  @Override
  public void initialize() {
    angleBen = angleEntry.getDouble(initialAngle);
    gyro.setYaw(angleBen + gyro.getAngle());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
