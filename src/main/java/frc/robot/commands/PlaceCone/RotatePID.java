// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

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
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.List;

public class RotatePID extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PoseEstimatorSubsystem poseEstimatorSubsystem;
  private LimeLightSub limeLight;
  private double tP = .02;
  private double tI = 0;
  private double tD = 0;

  private double tGoal;

  private double angleP = .03;
  private double angleI = 0;
  private double angleD = 0;

  private double angleGoal;

  private double anglePDefault = .03;
  private GenericEntry anglePEntry;

  private final ShuffleboardTab tab;

  private ProfiledPIDController anglePid;
  private ProfiledPIDController traversePid;

  // second param on constraints is estimated, should be max accel, not max speed, but lets say it
  // gets there in a second
  private Constraints angleConstraints =
      new Constraints(
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  private Constraints tConstraints =
      new Constraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 10,
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 10);

  public RotatePID(
      DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limeLight = limeLightSub;
    angleGoal = 180;
    tGoal = 0;

    tab = Shuffleboard.getTab("Drive");

    ShuffleboardLayout PIDConstants =
        tab.getLayout("Rotate PID Constants", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(8, 0);

    if (PIDConstants.getComponents().isEmpty()) {
      anglePEntry =
          PIDConstants.add("Angle P Constant", anglePDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();

    } else {
      List<ShuffleboardComponent<?>> widgets = PIDConstants.getComponents();
      anglePEntry = ((SimpleWidget) widgets.get(0)).getEntry();
    }
    addRequirements(poseEstimatorSubsystem, drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleP = anglePEntry.getDouble(anglePDefault);

    anglePid = new ProfiledPIDController(angleP, angleI, angleD, angleConstraints);
    traversePid = new ProfiledPIDController(tP, tI, tD, tConstraints);

    // configure rotation pid
    System.out.println(poseEstimatorSubsystem.getAngle());
    anglePid.reset(poseEstimatorSubsystem.getAngle());
    anglePid.setGoal(angleGoal);
    anglePid.setTolerance(0.3);
    anglePid.enableContinuousInput(-180, 180);

    traversePid.reset(limeLight.getTx());
    traversePid.setGoal(tGoal);
    traversePid.setTolerance(0.3);
    traversePid.disableContinuousInput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets drive values to align to 0 degrees, field oriented.
    double[] values = chassisValuesLower();
    drivetrainSubsystem.drive(new ChassisSpeeds(values[0], values[1], values[2]));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(.001, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return traversePid.atGoal() && anglePid.atGoal();
  }

  public double[] chassisValuesLower() {
    double[] x = new double[3];
    x[0] = 0.00001;
    x[1] = traversePid.calculate(limeLight.getTx());
    x[2] = anglePid.calculate(poseEstimatorSubsystem.getAngle());
    System.out.printf("Traverse %.2f, Rotate %.2f", x[1], x[2]);
    return x;
  }
}
