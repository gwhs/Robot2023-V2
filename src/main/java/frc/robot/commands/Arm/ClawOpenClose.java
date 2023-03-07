// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.List;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.ArmSubsystems.Claw;

public class ClawOpenClose extends CommandBase {

  private Claw claw;

  private final ShuffleboardTab tab;

  private double angle;
  private final GenericEntry desiredAngleEntry;
  private double desiredAngleDefault;

  private int amps;
  private final GenericEntry ampsEntry;
  private double ampsDefault;

  /**
   * The actual desired angle and amps will be pulled from the Shuffleboard. The
   * initial values displayed on the Shuffleboard will be the default values. In the case of missing
   * or unretrievable values, the command will use the default values.
   *
   * @param desiredAngleDefault The default desired angle. 📐
   * @param ampsDefault The default amps. (●'◡'●)
   */
  public ClawOpenClose(double desiredAngleDefault, int ampsDefault, Claw claw) {
    this.claw = claw;
    this.desiredAngleDefault = desiredAngleDefault;
    this.ampsDefault = ampsDefault;

    tab = Shuffleboard.getTab("Arm");

    ShuffleboardLayout openCloseShit =
        tab.getLayout("Claw Open Close Shit", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0);

    if (openCloseShit.getComponents().isEmpty()) {
      desiredAngleEntry =
          openCloseShit
              .add("Desired Angle", desiredAngleDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      ampsEntry =
          openCloseShit
              .add("Amps", ampsDefault)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
    } else {
      List<ShuffleboardComponent<?>> widgets = openCloseShit.getComponents();
      desiredAngleEntry = ((SimpleWidget) widgets.get(0)).getEntry();
      ampsEntry = ((SimpleWidget) widgets.get(1)).getEntry();
    }

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = desiredAngleEntry.getDouble(desiredAngleDefault);
    amps = (int) ampsEntry.getDouble(ampsDefault);
    claw.setLimit(amps);
    claw.setPosition(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    claw.brake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
