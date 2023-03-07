// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class Rotate extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PoseEstimatorSubsystem poseEstimatorSubsystem;
  private LimeLightSub limeLight;
  private double degrees;
  private double[] values = {0, 0, 0};
  private boolean angleDone = false;
  private boolean sideDone = false;
  private double p = .02;

  private double angleP;
  private double anglePDefault;
  private GenericEntry anglePEntry;

  private final ShuffleboardTab tab;

  private ProfiledPIDController anglePid;


  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  //// second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second

  
  private double angleI = 0;
  private double angleD = 0;

  

  /** Creates a new Rotate. */
  public Rotate(
      DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      LimeLightSub limeLightSub,
      double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limeLight = limeLightSub;
    this.degrees = degrees;

    anglePDefault = 3;

    tab = Shuffleboard.getTab("Drive");

    ShuffleboardLayout PIDConstants = tab.getLayout("Rotate PID Constants", BuiltInLayouts.kList).withSize(2, 4)
        .withPosition(2, 0);

    if (PIDConstants.getComponents().isEmpty()) {

      anglePEntry = PIDConstants.add("Angle P Constant", anglePDefault).withWidget(BuiltInWidgets.kTextView).getEntry();


    } else {
      List<ShuffleboardComponent<?>> widgets = PIDConstants.getComponents();
      anglePEntry = ((SimpleWidget) widgets.get(0)).getEntry();

    }

    addRequirements(poseEstimatorSubsystem, drivetrainSubsystem);
    // addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDone = false;

    angleP = anglePEntry.getDouble(anglePDefault);

    
anglePid =
    new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);

    // configure rotation pid
    System.out.println(poseEstimatorSubsystem.getAngle());
    anglePid.reset(Math.toRadians(poseEstimatorSubsystem.getAngle()));
    anglePid.setGoal(Math.toRadians(degrees));
    anglePid.setTolerance(Math.toRadians(.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets drive values to align to 0 degrees, field oriented.
    values = chassisValuesLower();
    drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
    // System.out.printf(
    //     "X equals %.2f PID moves %.2f%n", poseEstimatorSubsystem.getAngle(), values[2]);
    // setpoint and atgoal don't work, just brute forced.
    if (Math.abs(180 - poseEstimatorSubsystem.getAngle()) < .5) {
      angleDone = true;
    } else {
      angleDone = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Rotated done");
    drivetrainSubsystem.drive(new ChassisSpeeds(.001, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleDone;
  }

  public double[] chassisValuesLower() {
    /*
    [1,2,3]
    1 is x velocity
    2 is y velocity
    3 is degrees rotation
    */

    double[] x = new double[3];
    double d = anglePid.calculate(Math.toRadians(poseEstimatorSubsystem.getAngle()));
    x[0] = 0.00001;
    x[1] = Math.abs(limeLight.getTx()) > .5 ? (d > 0 ? -.3 : .3) : 0;
    x[2] = angleDone ? 0 : (d);
    return x;
  }
}
