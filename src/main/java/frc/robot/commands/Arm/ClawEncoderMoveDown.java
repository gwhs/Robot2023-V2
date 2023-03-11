// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;

public class ClawEncoderMoveDown extends CommandBase {

  private Claw clawOne;
  private BoreEncoder encoder;
  private double desiredAngle;
  private double error;

  private String piece;

  public ClawEncoderMoveDown(double angle, Claw initClaw, BoreEncoder weewoo, String piece) {
    clawOne = initClaw;
    this.encoder = weewoo;
    this.desiredAngle = angle;
    this.piece = piece;

    addRequirements(initClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("--------------START-----------");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = (-encoder.getRaw() / 8192. * 360.);
    error = (desiredAngle - rawAngle);
    double velocity = Constants.Claw.kP * error;
    if (velocity > Constants.Claw.DOWN_MAX_VELOCITY) {
      velocity = Constants.Claw.DOWN_MAX_VELOCITY;
    } else if (velocity < -Constants.Claw.DOWN_MAX_VELOCITY) {
      velocity = -Constants.Claw.DOWN_MAX_VELOCITY;
    }

    // System.out.println("DesiredAngle: " + desiredAngle);
    System.out.println("RawAngle: " + rawAngle);
  
    // System.out.println("Error: " + error);
    // System.out.println("Velocity: " + velocity);

    clawOne.setPercent(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    clawOne.setPercent(0);
    // System.out.println("---------------END------------");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1;
  }
}
