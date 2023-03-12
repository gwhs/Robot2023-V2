// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabPiece extends SequentialCommandGroup {
  /** Creates a new GrabPiece. */
  public GrabPiece(
      MagicMotion mainArm,
      BoreEncoder shaftEncoder,
      Claw clawPivot,
      BoreEncoder clawEncoder,
      Claw clawOpenClose,
      int mode) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (mode == 1) {
      addCommands(
          Commands.either(
              new ClawEncoderMoveDown(-125.0, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              Commands.sequence(
                  Commands.print("Encoder Pos" + -clawEncoder.getRaw() / 8192. * 360.),
                  Commands.parallel(
                      new ClawOpenCloseShuffleBoard(75, 5, clawOpenClose), Commands.waitSeconds(1)),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
                  new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
              clawEncoder::posDown));
    } else {
      Commands.sequence(
          Commands.print("START"),
          // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
          // Commands.waitSeconds(.25),
          // new MagicMotionPos(mainArm, 40, 1, 1, 5),
          new MagicMotionPosShuffleboard(mainArm, 175, 2.75, 5, shaftEncoder),
          // Commands.waitSeconds(.1),
          // new MagicMotionPosShuffleboard(mainArm, 180, 1, 1),
          // Commands.waitSeconds(),
          new MagicMotionPos(mainArm, 0, 3, 1.5, .5),
          Commands.waitSeconds(.5),
          // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
          // Commands.waitSeconds(.3),
          new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5));
    }
  }
}
