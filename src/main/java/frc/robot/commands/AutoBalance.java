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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GyroMoment.WrappedGyro;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoBalance extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private Timer engageTimer; // timer that counts how long the robot is 'engaged'
  private Timer stateChangeTimer;

  private double pConstant; // proportional constant in pid thing
  private double dConstant; // derivative constant in pid thing
  private double tolerance; // degrees within 0 to count the robot as being 'engaged'
  private double requiredEngageTime; // how many seconds to be engaged before stopping command
  private double requiredStateChangeTime;
  private double maxSpeed; // the fastest the robot can go (idk the units)
  private double initialSpeed; // speed of the robot in its 1st state

  private double pConstantDefault;
  private double dConstantDefault;
  private double maxSpeedDefault;
  private double requiredEngageTimeDefault;
  private double requiredStateChangeTimeDefault;
  private double toleranceDefault;
  private double initialSpeedDefault;

  private int state; // state of the robot; 0 or 1
  private double epsilonRollRate; // degrees per second threshold to switch from state 0 to 1
  private double epsilonRollRateDefault;
  private final GenericEntry epsilonRollRateEntry;

  private final ShuffleboardTab tab;
  private final GenericEntry pConstantEntry;
  private final GenericEntry dConstantEntry;
  private final GenericEntry toleranceEntry;
  private final GenericEntry maxSpeedEntry;
  private final GenericEntry requiredEngageTimeEntry;
  private final GenericEntry requiredStateChangeTimeEntry;
  private final GenericEntry initialSpeedEntry;

  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;

    pConstantDefault = 0.0014; // 0.0045 original these are the default values set on the robot and
    // shuffleboard
    dConstantDefault = 0.00008;
    toleranceDefault = 2.5;
    maxSpeedDefault = 0.5;
    requiredEngageTimeDefault = 0.1;
    requiredStateChangeTimeDefault = 0.008;
    initialSpeedDefault = 0.5;
    epsilonRollRateDefault = 13;

    engageTimer = new Timer();
    engageTimer.stop();
    stateChangeTimer = new Timer();
    stateChangeTimer.stop();
    tab = Shuffleboard.getTab("Auto Balance");

    ShuffleboardLayout orientation =
        tab.getLayout("Robot Orientation", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    ShuffleboardLayout input =
        tab.getLayout("Constant Inputs", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);

    WrappedGyro gryo = drivetrainSubsystem.getGyro();

    if (orientation.getComponents().isEmpty()) {
      orientation.addNumber("Yaw", () -> gryo.getYaw());
      orientation.addNumber("Pitch", () -> gryo.getPitch());
      orientation.addNumber("Roll", () -> gryo.getRoll());
      orientation.addNumber("Yaw Rate", () -> gryo.getYawRate());
      orientation.addNumber("Pitch Rate", () -> gryo.getPitchRate());
      orientation.addNumber("Roll Rate", () -> gryo.getRollRate());
    }

    if (input.getComponents().isEmpty()) {
      pConstantEntry =
          input.add("p Constant", pConstantDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
      dConstantEntry =
          input.add("d Constant", dConstantDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
      toleranceEntry =
          input.add("Tolerance", toleranceDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
      maxSpeedEntry =
          input.add("Max Speed", maxSpeedDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
      requiredEngageTimeEntry =
          input
              .add("Required Engage Time", requiredEngageTimeDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      requiredStateChangeTimeEntry =
          input
              .add("Required State Change Time", requiredStateChangeTimeDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      initialSpeedEntry =
          input
              .add("Initial State Speed", initialSpeedDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      epsilonRollRateEntry =
          input
              .add("Epsilon Roll Rate Threshold", epsilonRollRateDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
    } else {
      List<ShuffleboardComponent<?>> widgets = input.getComponents();
      pConstantEntry = ((SimpleWidget) widgets.get(0)).getEntry();
      dConstantEntry = ((SimpleWidget) widgets.get(1)).getEntry();
      toleranceEntry = ((SimpleWidget) widgets.get(2)).getEntry();
      maxSpeedEntry = ((SimpleWidget) widgets.get(3)).getEntry();
      requiredEngageTimeEntry = ((SimpleWidget) widgets.get(4)).getEntry();
      requiredStateChangeTimeEntry = ((SimpleWidget) widgets.get(5)).getEntry();
      initialSpeedEntry = ((SimpleWidget) widgets.get(6)).getEntry();
      epsilonRollRateEntry = ((SimpleWidget) widgets.get(7)).getEntry();
    }

    Logger.getInstance().start();

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    // each time the command is activated, it takes the values from the
    // shuffleboard---easier to
    // test values
    requiredEngageTime = requiredEngageTimeEntry.getDouble(requiredEngageTimeDefault);
    requiredStateChangeTime =
        requiredStateChangeTimeEntry.getDouble(requiredStateChangeTimeDefault);
    maxSpeed = maxSpeedEntry.getDouble(maxSpeedDefault);
    pConstant = pConstantEntry.getDouble(pConstantDefault);
    dConstant = dConstantEntry.getDouble(dConstantDefault);
    tolerance = toleranceEntry.getDouble(toleranceDefault);
    initialSpeed = initialSpeedEntry.getDouble(initialSpeedDefault);
    epsilonRollRate = epsilonRollRateEntry.getDouble(epsilonRollRateDefault);
    // prints values used in autobalance in the console
    System.out.printf(
        "max = %f, p Constant = %f, d Constant = %f, tolerance = %f, Required Engage Time = %f, Initial p Constant = %f, Epsilon Roll Rate Threshold = %f, Required State Change Time = %f",
        maxSpeed,
        pConstant,
        dConstant,
        tolerance,
        requiredEngageTime,
        initialSpeed,
        epsilonRollRate,
        requiredStateChangeTime);

    engageTimer.reset();
    engageTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    WrappedGyro gyro = drivetrainSubsystem.getGyro();
    double currentAngle = gyro.getRoll();
    double currentDPS = gyro.getRollRate();

    double error = currentAngle - 0;

    Logger.getInstance()
        .recordOutput("Pitch (actually roll cuz pigeon is oriented weirdly or smth", currentAngle);
    Logger.getInstance()
        .recordOutput(
            "Pitch Rate (actually roll cuz pigeon is oriented weirdly or smth", currentDPS);

    double speed = 0;

    // sometimes currentDPS spikes because it reads somehting weirdly do this later
    if (Math.abs(currentDPS) >= Math.abs(epsilonRollRate)) {
      stateChangeTimer.start();
    } else {
      stateChangeTimer.stop();
      stateChangeTimer.reset();
    }

    if (stateChangeTimer.hasElapsed(requiredStateChangeTime)) {
      state = 1;
    }

    if (state == 0) {
      speed = Math.copySign(initialSpeed, error);
    }

    if (state == 1) {
      speed = error * pConstant + currentDPS * dConstant;
    }

    if (speed > maxSpeed) {
      speed = maxSpeed;
    } else if (speed < -maxSpeed) {
      speed = -maxSpeed;
    }

    Logger.getInstance().recordOutput("Speed", speed);

    if (Math.abs(error) <= tolerance) {
      engageTimer.start();
    } else {
      engageTimer.stop();
      engageTimer.reset();
    }

    Logger.getInstance().recordOutput("Engage Timer", engageTimer.get());

    drivetrainSubsystem.drive(new ChassisSpeeds(speed, 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
    Logger.getInstance().end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return engageTimer.hasElapsed(requiredEngageTime);
  }
}
