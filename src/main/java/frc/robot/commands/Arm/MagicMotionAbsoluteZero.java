// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;

public class MagicMotionAbsoluteZero extends CommandBase {
  private MagicMotion motor;
  private BoreEncoder encoder;
  private double velocity;
  private double acceleration;
  private double rawAngle;
  private double motorAngle;
  private double difference;
  /** Creates a new MagicMotionAbsoluteZero. */
  public MagicMotionAbsoluteZero(
      MagicMotion motor, BoreEncoder encoder, double velocity, double acceleration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = motor;
    this.encoder = encoder;
    this.acceleration = acceleration;
    this.velocity = velocity;
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("-------RUNNING MMAZ----------");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorAngle = motor.getAngDegrees();
    rawAngle = encoder.getRaw() / 8192. * 360.;
    double difference = rawAngle - motorAngle;
    System.out.println("Difference: " + difference);
    System.out.println("Raw:" + rawAngle);
    System.out.println("Motor Angle:" + motorAngle);
    // System.out.println(encoder.getRaw());
    motor.setAng(-difference, velocity, acceleration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rawAngle = encoder.getRaw() / 8192. * 360.;
    System.out.println("Raw:" + rawAngle);
    motor.enableBrakeMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    rawAngle = encoder.getRaw() / 8192.0 * 360;
    return Math.abs(rawAngle) < .5;
  }
}
