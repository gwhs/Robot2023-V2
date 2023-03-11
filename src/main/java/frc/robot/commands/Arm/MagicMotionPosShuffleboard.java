// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import java.util.List;

public class MagicMotionPosShuffleboard extends CommandBase {
  /** Creates a new mMPosUp. */
  private MagicMotion motor;
  // private double offset; // motor keeps moving after end, so we get an offset
  // to make sure the
  // motor returns to the same position each time
  // private boolean ran; // ensures the motor shoots, otherwise, it will not run
  // after one shot,
  // needed for isFinished
  // private double returnSpeed;
  // private double amps;

  private double motorAng;

  private final ShuffleboardTab tab;

  private double angle;
  private final GenericEntry desiredAngleEntry;
  private double desiredAngleDefault;

  private double velocity;
  private final GenericEntry velocityEntry;
  private double velocityDefault;
  private BoreEncoder boreEncoder;
  private double acceleration;
  private final GenericEntry accelerationEntry;
  private double accelerationDefault;

  /**
   * The actual desired angle, velocity, and acceleration will be pulled from the Shuffleboard. The
   * initial values displayed on the Shuffleboard will be the default values. In the case of missing
   * or unretrievable values, the command will use the default values.
   *
   * @param desiredAngleDefault The default desired angle. 📐
   * @param velocityDefault The default velocity. (●'◡'●)
   * @param accelerationDefault The default acceleration. ☆*: .｡. o(≧▽≦)o .｡.:*☆
   */
  public MagicMotionPosShuffleboard(
      MagicMotion motor,
      double desiredAngleDefault,
      double velocityDefault,
      double accelerationDefault,
      BoreEncoder encoder) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.motor = motor;
    this.desiredAngleDefault = desiredAngleDefault;
    this.velocityDefault = velocityDefault;
    this.accelerationDefault = accelerationDefault;
    this.boreEncoder = encoder;
    tab = Shuffleboard.getTab("Arm");

    ShuffleboardLayout magicMotionConstants =
        tab.getLayout("Magic Motion Constants", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0);

    if (magicMotionConstants.getComponents().isEmpty()) {
      desiredAngleEntry =
          magicMotionConstants
              .add("Desired Angle", desiredAngleDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      velocityEntry =
          magicMotionConstants
              .add("Velocity", velocityDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      accelerationEntry =
          magicMotionConstants
              .add("Acceleration", accelerationDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
    } else {
      List<ShuffleboardComponent<?>> widgets = magicMotionConstants.getComponents();
      desiredAngleEntry = ((SimpleWidget) widgets.get(0)).getEntry();
      velocityEntry = ((SimpleWidget) widgets.get(1)).getEntry();
      accelerationEntry = ((SimpleWidget) widgets.get(2)).getEntry();
    }

    
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("mMPos");
    angle = desiredAngleEntry.getDouble(desiredAngleDefault);
    velocity = velocityEntry.getDouble(velocityDefault);
    acceleration = accelerationEntry.getDouble(accelerationDefault);
    motor.enableBrakeMode(true);
    boreEncoder.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setAng(angle, velocity, acceleration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    motorAng = motor.getAngDegrees();
    return Math.abs(motorAng - angle) < 5;
  }
}
