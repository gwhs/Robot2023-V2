// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;

public class ClawEncoderMoveDownPID extends ProfiledPIDCommand {

  public ClawEncoderMoveDownPID(
      double angle, Claw initClaw, BoreEncoder clawBoreEncoder, String piece) {

    super(
        new ProfiledPIDController(
            Constants.Claw.kP,
            Constants.Claw.kI,
            Constants.Claw.kD,
            new TrapezoidProfile.Constraints(Constants.Claw.DOWN_MAX_VELOCITY, 1)),
        () -> -clawBoreEncoder.getAngle(),
        () -> angle,
        (value, state) -> {
          initClaw.setPercent(value);
          System.out.println("piece: " + piece + " RawAngle: " + (-clawBoreEncoder.getAngle()));
        },
        initClaw,
        clawBoreEncoder);
    getController().setTolerance(1, 20);
    Shuffleboard.getTab("PID setup").add("Claw Down PID", getController());
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
