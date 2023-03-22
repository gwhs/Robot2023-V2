// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.Arm.ClawEncoderMoveDown;
import frc.robot.commands.Arm.ClawEncoderMoveUp;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PlaceLow extends SequentialCommandGroup {
  /** Creates a new PlaceLow. */
  public PlaceLow(
      DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      LimeLightSub limeLightSub,
      MagicMotion mainArm,
      BoreEncoder shaftEncoder,
      BoreEncoder clawEncoder,
      Claw clawPivot,
      int degrees) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PPIDAutoAim(drivetrainSubsystem, limeLightSub, LimeLightConstants.LOWER_DISTANCE_SHOOT),
        new Rotate(drivetrainSubsystem, poseEstimatorSubsystem, limeLightSub),
        new ClawEncoderMoveDown(-30.0, clawPivot, clawEncoder, "Cube").withTimeout(.1),
        Commands.waitSeconds(.1),
        new MagicMotionPos(mainArm, degrees, 1, 1, .5),
        Commands.waitSeconds(.1),
        new MagicMotionPos(mainArm, 2, 1, 1, .5),
        Commands.waitSeconds(.5),
        new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
        Commands.waitSeconds(.3),
        new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5));
  }
}
