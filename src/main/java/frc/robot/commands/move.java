// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class move extends CommandBase {
  private Arm motor;
  // private double offset; // motor keeps moving after end, so we get an offset to make sure the
  // motor returns to the same position each time
  // private boolean ran; // ensures the motor shoots, otherwise, it will not run after one shot,
  // needed for isFinished
  // private double returnSpeed;
  // private double amps;
  private double speed;
  private boolean isInterrupted;

  public move(Arm moto, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = moto;
    this.speed = speed;

    addRequirements(moto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Testing");
    isInterrupted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setPercent(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
