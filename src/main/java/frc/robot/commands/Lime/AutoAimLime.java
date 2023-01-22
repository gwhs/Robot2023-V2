// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lime;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.Constants;

public class AutoAimLime extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private boolean pos;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limeLight.getTx() >= 0){
      pos = true;
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(-20)));
    } else{
      pos = false;
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(20)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Final X-error" ,limeLight.getTx());
    SmartDashboard.putNumber("Final Y-error", limeLight.getTy());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limeLight.getTx() >= -LimeLightConstants.MAX_LIMELIGHT_ERROR_DEGREES && limeLight.getTx() <= LimeLightConstants.MAX_LIMELIGHT_ERROR_DEGREES) {
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
      return true;
    }
    
    return false;
  }
}
