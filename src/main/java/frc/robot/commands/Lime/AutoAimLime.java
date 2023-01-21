// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lime;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class AutoAimLime extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private boolean done = false;
  private double targetX = 1;
  private double targetY = LimeLightConstants.MAX_LIMELIGHT_ERROR_DEGREES;

  /** Creates a new AutoAimLime. */
  public AutoAimLime(DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLightSub;
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (limeLight.getTx() < 0) {
    //   while (limeLight.getTx() < targetX) {
    //     drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(10)));
    //   }
    // } else {
    //   while (Math.abs(limeLight.getTx()) > targetX) {
    //     drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(-10)));
    //   }
    // }
    // drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
    // done = true;
    if (limeLight.getTx() < 0) {
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(10)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limeLight.getTx() > 0) {
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
      return true;
    }
    return false;
  }
}
