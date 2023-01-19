// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lime;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveTrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoAimLime extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  double degreesToTarget;

  /** Creates a new AutoAimLime. */
  public AutoAimLime(DrivetrainSubsystem drivetrainSubsystem, double degreesToTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.degreesToTarget = degreesToTarget;
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
