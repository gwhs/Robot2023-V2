package frc.robot.commands.LimelightFollow;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.GenericEntry;

public class PPIDAutoFollow extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private LimeLightSub limeLight;
  private double[] values = {0, 0, 0};
  private boolean sidewaysDone = false;
 
  private double positionPDefault;
  private double distanceError;
  private int noTargets = 0;

  private GenericEntry positionPEntry;

  private Constraints positionConstraints =
  new Constraints(
      DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50,
      DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50);


  private double targetDistance = 0;
  private double positionP = .007;


  public PPIDAutoAim(
    DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub, double targetDistance) {
  // Use addRequirements() here to declare subsystem dependencies.
  this.limeLight = limeLightSub;
  this.drivetrainSubsystem = drivetrainSubsystem;
  this.targetDistance = targetDistance;

  positionPDefault = 0.025;

  addRequirements(limeLight, drivetrainSubsystem);
  }
  
  @Override
  public void initialize() {
    distanceError = limeLightSub.getTx() - targetDistance; 

  }

  @Override
  public void execute() {

    if (limeLight.hasTarget()) {
      // calculates drive values, pid.calculate called in this function
      noTargets = 0;
      values = chassisValuesLower();
      // makes it drive!
      drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
    } else {
      noTargets++;
    }

    if (Math.abs(distanceError) < 2) {
      sidewaysDone = true;
    } else {
      // sets to false if angle not there yet
      sidewaysDone = false;
    }

    if (noTargets >= 10) {
      sidewaysDone = true;
      angleDone = true;
      System.out.println("target is gone!");
      
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("done");
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {

    return angleDone && sidewaysDone;
  }
}