package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.PPSwerveFollower;
import frc.robot.commands.Arm.ClawEncoderMoveDown;
import frc.robot.commands.Arm.ClawEncoderMoveUp;
import frc.robot.commands.Arm.ClawOpenClose;
import frc.robot.commands.Arm.ClawOpenCloseShuffleBoard;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.PlaceCone.PlaceHigh;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.Claw;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public final class TestAutoCommands {

  private DrivetrainSubsystem driveSystem;
  private PoseEstimatorSubsystem poseEstimatorSystem;
  private MagicMotion mainArm;
  private BoreEncoder shaftEncoder;
  private String pathName;
  private LimeLightSub lime;
  private BoreEncoder clawEncoder;
  private Claw clawPivot;
  private Claw clawOpenClose;

  public TestAutoCommands(
      DrivetrainSubsystem d,
      PoseEstimatorSubsystem poseEstimatorSystem,
      MagicMotion m,
      BoreEncoder b,
      String p,
      LimeLightSub l,
      BoreEncoder c,
      Claw clawP,
      Claw clawOP) {
    this.driveSystem = d;
    this.poseEstimatorSystem = poseEstimatorSystem;
    this.mainArm = m;
    this.shaftEncoder = b;
    this.pathName = p;
    this.lime = l;
    this.clawEncoder = c;
    this.clawPivot = clawP;
    this.clawOpenClose = clawOP;
  }

  public SequentialCommandGroup starting(String path, double maxVel, double maxAcc) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
            new PPSwerveFollower(
                driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
        new MagicMotionPos(mainArm, 40, 1, 1, .5),
        new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
        new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
        Commands.waitSeconds(.25),
        new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
        new ParallelCommandGroup(
            new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
            new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
            new PPSwerveFollower(
                driveSystem,
                poseEstimatorSystem,
                path,
                new PathConstraints(maxVel, maxAcc),
                true)));
  }

  public SequentialCommandGroup getAutoCommand() {
    if (pathName.equals("StraightNoRotation")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem,
              poseEstimatorSystem,
              "StraightNoRotation",
              new PathConstraints(1, 1),
              true),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(clawEncoder::posDown, clawEncoder));
    }

    if (pathName.equals("TestCurve")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "TestCurve", new PathConstraints(1, 1), true));
    }
    if (pathName.equals("StraightWithRotation")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem,
              poseEstimatorSystem,
              "StraightWithRotation",
              new PathConstraints(1, 1),
              true));
    }
    if (pathName.equals("HajelPath")) {
      return starting(pathName, 3, 2).andThen(new AutoBalance(driveSystem));
    }
    if (pathName.equals("HajelPathV2NoLime")) {
      return new SequentialCommandGroup(
          starting("I2+1", 3, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1Part2", new PathConstraints(1, 1), true),
          new ParallelCommandGroup(
              new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
          new MagicMotionPos(mainArm, 40, 1, 1, .5),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                  new PPSwerveFollower(
                      driveSystem,
                      poseEstimatorSystem,
                      "HajelPathV2Part3",
                      new PathConstraints(2, 1),
                      true))),
          new AutoBalance(driveSystem));
    }
    if (pathName.equals("HajelPathV2")) {
      return new SequentialCommandGroup(
          starting("I2+1", 3, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          new PlaceHigh(
              driveSystem,
              poseEstimatorSystem,
              lime,
              mainArm,
              shaftEncoder,
              clawEncoder,
              clawPivot,
              190),
          new PPSwerveFollower(
              driveSystem,
              poseEstimatorSystem,
              "HajelPathV2Part3",
              new PathConstraints(2, 1),
              true),
          new AutoBalance(driveSystem));
    }
    if (pathName.equals("D-F1E")) {
      return starting(pathName, 2, 2).andThen(new AutoBalance(driveSystem));
    }
    if (pathName.equals("A2E")) {
      return new SequentialCommandGroup(
          starting(pathName, 2, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "A2EPart2", new PathConstraints(2, 2), true),
          Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          new PlaceHigh(
              driveSystem,
              poseEstimatorSystem,
              lime,
              mainArm,
              shaftEncoder,
              clawEncoder,
              clawPivot,
              190),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "A2EPart3", new PathConstraints(2, 2), true),
          new AutoBalance(driveSystem));
    }
    if (pathName.equals("D1+1")) {
      return new SequentialCommandGroup(
          starting(pathName, 2, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)));
    }
    if (pathName.equals("F1+1")) {
      return new SequentialCommandGroup(
          starting(pathName, 1, 1),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)));
    }
    if (pathName.equals("G2E")) {
      return new SequentialCommandGroup(
          starting(pathName, 2, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          new PlaceHigh(
              driveSystem,
              poseEstimatorSystem,
              lime,
              mainArm,
              shaftEncoder,
              clawEncoder,
              clawPivot,
              190),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "G2EPart3", new PathConstraints(2, 2), true),
          new AutoBalance(driveSystem));
    }
    if (pathName.equals("G2ENoLime")) {
      return new SequentialCommandGroup(
          starting("G2E", 2, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "G2EPart2", new PathConstraints(1, 2), true),
          new ParallelCommandGroup(
              new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
          new MagicMotionPos(mainArm, 40, 1, 1, .5),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                  new PPSwerveFollower(
                      driveSystem,
                      poseEstimatorSystem,
                      "G2EPart3",
                      new PathConstraints(2, 2),
                      true))),
          new AutoBalance(driveSystem));
    }
    if (pathName.equals("I2+1")) {
      return new SequentialCommandGroup(
          starting(pathName, 2, 2),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          new PlaceHigh(
              driveSystem,
              poseEstimatorSystem,
              lime,
              mainArm,
              shaftEncoder,
              clawEncoder,
              clawPivot,
              190),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1Part3", new PathConstraints(2, 2), true),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)));
    }
    if (pathName.equals("I2NoLime")) {
      return new SequentialCommandGroup(
          starting("I2+1", 1.5, 1),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1Part2", new PathConstraints(1.5, 1), true),
          new ParallelCommandGroup(
              new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
          new MagicMotionPos(mainArm, 40, 1, 1, .5),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"))));
    }
    if (pathName.equals("I2")) {
      return new SequentialCommandGroup(
          starting("I2+1", 1, 1),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(clawEncoder::posDown, clawEncoder),
          Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          new PlaceHigh(
              driveSystem,
              poseEstimatorSystem,
              lime,
              mainArm,
              shaftEncoder,
              clawEncoder,
              clawPivot,
              190));
    }

    if (pathName.equals("I2+1E")) {
      return new SequentialCommandGroup(
          starting("I2+1", 1, 1),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          new PlaceHigh(
              driveSystem,
              poseEstimatorSystem,
              lime,
              mainArm,
              shaftEncoder,
              clawEncoder,
              clawPivot,
              190),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1Part3", new PathConstraints(2, 2), true),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1E", new PathConstraints(2, 2), true),
          new AutoBalance(driveSystem));
    }
    if (pathName.equals("C1+E")) {
      return starting(pathName, 3, 2).andThen(new AutoBalance(driveSystem));
    }
    if (pathName.equals("G1+E")) {
      return starting(pathName, 3, 2).andThen(new AutoBalance(driveSystem));
    }
    return null;
  }
}
