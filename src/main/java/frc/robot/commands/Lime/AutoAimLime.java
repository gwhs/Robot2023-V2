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

public class AutoAimLime extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values;
  private boolean horizDone = false;
  private boolean angleDone = false;
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
    horizDone = false;
    angleDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // add pids
    values = chassisValuesLower();
    if (limeLight.hasTarget()) {
      drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
    } else {
      System.out.println("NO TARGET FOUND BY LIMELIGHT");
    }
    if (Math.abs(limeLight.getXDistance() - LimeLightConstants.LOWER_DISTANCE_SHOOT) < 2) {
      horizDone = true;
    } else {
      horizDone = false;
    }
    if (Math.abs(limeLight.getTx()) < 2) {
      angleDone = true;
    } else {
      angleDone = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Final Horiz-error", limeLight.getTx());
    SmartDashboard.putNumber("Final Vert-error", limeLight.getTy());
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // && angleDone
    return angleDone && horizDone;
  }

  public double[] chassisValuesLower() {
    /*
    [1,2,3]
    1 is x velocity
    2 is y velocity
    3 is degrees rotation
    get the angle using atan2, it returns radians
    use sin and cos to get values to reach max speed
    not really sure about the angle yet.
    */
    double distanceError = limeLight.getXDistance() - LimeLightConstants.LOWER_DISTANCE_SHOOT;
    double[] x = new double[3];
    x[0] = distanceError / 10;
    x[1] = 0;
    x[2] = -limeLight.getTx();
    // getAngle()
    //     / (((Math.sqrt(x[0] * x[0]) + x[1] * x[1]))
    //         / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    return x;
  }
}
