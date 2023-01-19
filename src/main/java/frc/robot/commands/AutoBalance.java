// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private double currentAngle;
  private double currentDPS;
  private double speed;
  private double pConstant;
  private double dConstant;
  private Timer timer;
  private double error;
  private final double TOLERANCE;
  private final double DESIRED_ENGAGE_TIME; //in milliseconds

  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    pConstant = 0.008;
    dConstant = -0.01;
    timer = new Timer();
    timer.stop();
    TOLERANCE = 2.5; //in degrees
    DESIRED_ENGAGE_TIME = 1; //in seconds

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = drivetrainSubsystem.getRoll();
    currentDPS = drivetrainSubsystem.getRollRate();

    error = currentAngle - 0;

    speed = error * pConstant + currentDPS * dConstant;

    if (speed > 0.2)
    {
      speed = 0.2;
    }
    else
    {

    }

    if (Math.abs(error) <= TOLERANCE)
    {
      timer.start();
    }
    else
    {
      timer.stop();
      timer.reset();
    }

    drivetrainSubsystem.drive(new ChassisSpeeds(speed, 0.0, 0.0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(DESIRED_ENGAGE_TIME);
  }
}
