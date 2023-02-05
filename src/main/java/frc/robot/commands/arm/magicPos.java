// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MagicMotion;
 
public class magicPos extends CommandBase {
  private MagicMotion motor;
  // private double offset; // motor keeps moving after end, so we get an offset to make sure the
  // motor returns to the same position each time
  // private boolean ran; // ensures the motor shoots, otherwise, it will not run after one shot,
  // needed for isFinished
  // private double returnSpeed;
  // private double amps;
  private double angle;
  private double velocity;
  private double acceleration;
  private boolean isInterrupted;

  //Velocity -> Degrees per sec
  //Acceleration -> Degrees/sec per sec
  public magicPos(MagicMotion moto, double angle, double velocity, double acceleration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = moto;
    this.angle = angle;
    this.velocity = velocity;
    this.acceleration = acceleration;

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
    motor.setAng(angle, velocity, acceleration);
    System.out.println(motor.getAng() /(Constants.Arm.FALCON_TICKS * 36) * 360);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.setAng(0, velocity, acceleration);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
