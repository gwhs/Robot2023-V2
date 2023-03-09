// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.Claw;

public class CLAWPERCENT extends CommandBase {
  private Claw claw;

  private double angle;
  private int amps;

  public CLAWPERCENT(double desiredAngle, int desiredAmps, Claw claw) {
    this.claw = claw;
    this.angle = desiredAngle;
    this.amps = desiredAmps;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setPercent(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    claw.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
