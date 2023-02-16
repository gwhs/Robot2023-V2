// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GyroMoment.WrappedGyro;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ShuffleBoardBen extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  // private ShuffleboardTab tab = Shuffleboard.getTab("ShuffleBoardBen");

  private Timer engageTimer; // timer that counts how long the robot is 'engaged'

  private double pConstantDefault;
  private double dConstantDefault;
  private double maxSpeedDefault;
  private double desiredEngageTimeDefault;
  private double toleranceDefault;
  private GenericEntry initialSpeedDefault; // angle bop

  //  private int state; // state of the robot; 0 or 1
  private double epsilonRollRateDefault;

  /** Creates a new AutoBalance. */
  public ShuffleBoardBen(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    final ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    Shuffleboard.selectTab("Drive");

    initialSpeedDefault = tab.add("Current Angle", 20).getEntry();

    // Use addRequirements() here to declare subsystem dependencies.

    pConstantDefault = 0.0045; // these are the default values set on the robot and shuffleboard
    dConstantDefault = -0.0012;
    toleranceDefault = 2.5;
    maxSpeedDefault = 0.5;
    desiredEngageTimeDefault = 0.5;
    // initialSpeedDefault = 0.5; // controller angle
    epsilonRollRateDefault = 25;

    engageTimer = new Timer();
    engageTimer.stop();

    ShuffleboardLayout orientation =
        tab.getLayout("Robot Orientation", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    ShuffleboardLayout input =
        tab.getLayout("Constant Inputs", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);

    WrappedGyro gryo = drivetrainSubsystem.getGyro();
    orientation.addNumber("Yaw", () -> gryo.getYaw()); // gets angle
    orientation.addNumber("Pitch", () -> gryo.getPitch());
    orientation.addNumber("Roll", () -> gryo.getRoll());
    orientation.addNumber("Yaw Rate", () -> gryo.getYawRate());
    orientation.addNumber("Pitch Rate", () -> gryo.getPitchRate());
    orientation.addNumber("Roll Rate", () -> gryo.getRollRate());

    System.out.println(initialSpeedDefault);
  }
}
