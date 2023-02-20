// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StraightWheel extends CommandBase {
  /** Creates a new StraightWheel. */
  private DrivetrainSubsystem drivetrainSubsystem;

  private int counter = 0;
  private SwerveModulePosition[] positions;
  private SwerveModuleState[] swervemodule = new SwerveModuleState[4];

  public StraightWheel(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    swervemodule[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    swervemodule[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    swervemodule[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    swervemodule[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    drivetrainSubsystem.setModuleStates(swervemodule);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    positions = drivetrainSubsystem.getModulePositions();
    drivetrainSubsystem.setModuleStates(swervemodule);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 10;
  }
}
