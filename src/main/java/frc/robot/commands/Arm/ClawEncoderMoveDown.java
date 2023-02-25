// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ClawEncoderMoveDown extends CommandBase {

  private Claw clawOne;
  private BoreEncoder encoder;
  private double desiredAngle;
  private double error;

  public ClawEncoderMoveDown(double angle, Claw initClaw , BoreEncoder weewoo) {
    clawOne = initClaw;
    this.encoder = weewoo;
    this.desiredAngle = angle;
    addRequirements(initClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = (-encoder.getRaw() / 8192. * 360.);
    error = (desiredAngle - rawAngle);
    double velocity = Constants.Claw.kP * error;

    if(velocity > Constants.Claw.DOWN_MAX_VELOCITY){
      velocity = Constants.Claw.DOWN_MAX_VELOCITY;
    }
    else if(velocity < -Constants.Claw.DOWN_MAX_VELOCITY){
      velocity = -Constants.Claw.DOWN_MAX_VELOCITY;
    }

    clawOne.setPercent(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    clawOne.brake();
    clawOne.setPercent(0);
  }

  @Override 
  public boolean isFinished() {
    return Math.abs(error) < .5;
  }
}
