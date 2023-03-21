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
public class ArmSequenceTop {
  private MagicMotion mainArm;
  private BoreEncoder shaftEncoder;
  private BoreEncoder clawEncoder;
  private Claw clawPivot;
  /** Creates a new MasterConeSequence. */
  public ArmSequenceTop(MagicMotion m, BoreEncoder b, BoreEncoder c, Claw clawP) {
    this.mainArm = m;
    this.shaftEncoder = b;
    this.clawEncoder = c;
    this.clawPivot = clawP;
  }

  public SequentialCommandGroup starting() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println(mainArm.getMode());

    return new SequentialCommandGroup(
        Commands.either(
            Commands.sequence(
                Commands.print("START CONE "),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // Commands.waitSeconds(.25),
                Commands.runOnce(mainArm::resetPosition, mainArm),
                // new MagicMotionPos(mainArm, 40, 1, 1, 5),
                new MagicMotionPos(mainArm, 195.0, 2.75, 5.5, 1),
                Commands.waitSeconds(.25),
                new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)),
            Commands.sequence(
                Commands.print("START CUBE"),
                // new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
                // new PPIDAutoAim(drivetrainSubsystem, limeLightSub, 44),
                // Commands.waitSeconds(.25),
                Commands.runOnce(mainArm::resetPosition, mainArm),
                new MagicMotionPosShuffleboard(mainArm, 210, 2.75, 3.5, shaftEncoder),
                Commands.waitSeconds(.25),
                new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                // new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5)),
            mainArm::isConeMode));
  }
}
