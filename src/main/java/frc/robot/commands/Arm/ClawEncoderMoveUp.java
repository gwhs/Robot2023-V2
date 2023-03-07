// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;

public class ClawEncoderMoveUp extends CommandBase {

  private Claw clawOne;
  private BoreEncoder encoder;
  private double desiredAngle;
  private double error;

  private String piece;
  private GenericEntry pieceEntry;
  private String pieceDefault;

  // private final ShuffleboardTab tab;

  public ClawEncoderMoveUp(double angle, Claw initClaw, BoreEncoder weewoo, String piece) {
    clawOne = initClaw;
    this.encoder = weewoo;
    this.desiredAngle = angle;
    this.piece = piece;
    addRequirements(initClaw);

    // tab = Shuffleboard.getTab("Arm");

    // ShuffleboardLayout moveUp =
    //     tab.getLayout("Claw Encoder Move Up", BuiltInLayouts.kList)
    //         .withSize(2, 4)
    //         .withPosition(2, 0);

    // if (moveUp.getComponents().isEmpty()) {
    //   pieceEntry =
    //       moveUp.add("Piece Name", pieceDefault).withWidget(BuiltInWidgets.kTextView).getEntry();
    // } else {
    //   List<ShuffleboardComponent<?>> widgets = moveUp.getComponents();
    //   pieceEntry = ((SimpleWidget) widgets.get(0)).getEntry();
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("--------------START-----------");
    piece = pieceEntry.getString(pieceDefault);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = (-encoder.getRaw() / 8192. * 360.);
    error = (desiredAngle - rawAngle);
    double velocity = Constants.Claw.kP * error;
    if (piece.toUpperCase().equals("CUBE")) {
      if (velocity > Constants.Claw.CUBE_UP_MAX_VELOCITY) {
        velocity = Constants.Claw.CUBE_UP_MAX_VELOCITY;
      } else if (velocity < -Constants.Claw.CUBE_UP_MAX_VELOCITY) {
        velocity = -Constants.Claw.CUBE_UP_MAX_VELOCITY;
      }
    } else if (piece.toUpperCase().equals("CONE")) {
      if (velocity > Constants.Claw.CONE_UP_MAX_VELOCITY) {
        velocity = Constants.Claw.CONE_UP_MAX_VELOCITY;
      } else if (velocity < -Constants.Claw.CONE_UP_MAX_VELOCITY) {
        velocity = -Constants.Claw.CONE_UP_MAX_VELOCITY;
      }
    }

    clawOne.setPercent(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    clawOne.brake();
    clawOne.setPercent(0);
    System.out.println("CLAWMOVEUP FINISHED");
    // System.out.pr intln("---------------END------------");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1;
  }
}
