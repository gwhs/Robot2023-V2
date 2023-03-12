package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.commands.PlaceCone.ChangePipeline;
import frc.robot.commands.PlaceCone.PPIDAutoAim;
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

  public SequentialCommandGroup getAutoCommand() {
    if (pathName.equals("BenPath")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(1, 1), true))),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(clawEncoder::posDown, clawEncoder),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "BenPath", new PathConstraints(1, 1), true),
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
    if (pathName.equals("StraightWithRotation")) { // testing the rotation
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem,
              poseEstimatorSystem,
              "StraightWithRotation",
              new PathConstraints(1, 1),
              true));
    }
    if (pathName.equals("HajelPath")) {
      return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
          new MagicMotionPos(mainArm, 40, 1, 1, .5), // NEW
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
                      "HajelPath",
                      new PathConstraints(3, 2),
                      true))),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("HajelPathV2NoLime")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          new MagicMotionPos(mainArm, 190, 0, 0, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 0, 0, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(1, 1), true))),
          // Commands.waitSeconds(1),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1Part2", new PathConstraints(1, 1), true),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem,
                      poseEstimatorSystem,
                      "HajelPathV2Part3",
                      new PathConstraints(2, 1),
                      true))),
          // Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          // new PlaceHigh(driveSystem, poseEstimatorSystem, lime, mainArm, shaftEncoder, 190),

          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("HajelPathV2")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(5, 3), true))),
          // Commands.waitSeconds(1),
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
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    //   return new SequentialCommandGroup(
    //       new PPSwerveFollower(
    //           driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
    //       new MagicMotionPos(mainArm, 210, 0, 0),
    //       Commands.waitSeconds(.5),
    //       new ParallelCommandGroup(
    //           new SequentialCommandGroup(
    //               new MagicMotionPos(mainArm, 0, 0, 0),
    //               Commands.waitSeconds(.5),
    //               new MagicMotionAbsoluteZero(mainArm, shaftEncoder)),
    //           new PPSwerveFollower(
    //               driveSystem,
    //               poseEstimatorSystem,
    //               "HajelPathV2",
    //               new PathConstraints(1, 2),
    //               true)),
    //       Commands.waitSeconds(1), // grab
    //     //   new PPSwerveFollower(
    //     //       driveSystem,
    //     //       poseEstimatorSystem,
    //     //       "HajelPathV2Part2",
    //     //       new PathConstraints(1, 2),
    //     //       true),
    //     Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
    //     new PlaceHigh(driveSystem, poseEstimatorSystem, lime, mainArm, shaftEncoder, 190),
    //       new PPSwerveFollower(
    //         driveSystem,
    //         poseEstimatorSystem,
    //         "HajelPathV2Part3",
    //         new PathConstraints(1, 2),
    //         true)),
    //       new AutoBalance(driveSystem));
    // }
    if (pathName.equals("D-F1E")) {
      return new SequentialCommandGroup(
          new ClawEncoderMoveDown(-80, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          new PPIDAutoAim(driveSystem, lime, 44),
          new ChangePipeline(lime),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          Commands.waitSeconds(.5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "D-F1E", new PathConstraints(2, 2), true))),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("A2E")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          Commands.waitSeconds(.5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "A2E", new PathConstraints(2, 2), true))),
          // Commands.waitSeconds(1), // grab
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
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("D1+1")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "D1+1", new PathConstraints(2, 2), true))),
          //   Commands.waitSeconds(1) // grab
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
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "F1+1", new PathConstraints(1, 1), true))),
          // Commands.waitSeconds(1) // grab
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
          Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "G2E", new PathConstraints(2, 2), true))),
          // Commands.waitSeconds(1), // grab
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          // new PPSwerveFollower(
          // driveSystem, poseEstimatorSystem, "G2EPart2", new PathConstraints(1, 2), true),
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
          //   Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
          //   new AllLime(driveSystem, poseEstimatorSystem, lime, 0), // april tag?
          //   new MagicMotionPos(mainArm, 210, 0, 0),
          //   new ParallelCommandGroup(
          //       new SequentialCommandGroup(
          //           new MagicMotionPos(mainArm, 0, 0, 0),
          //           Commands.waitSeconds(.5),
          //           new MagicMotionAbsoluteZero(mainArm, shaftEncoder),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "G2EPart3", new PathConstraints(2, 2), true), // )),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("G2ENoLime")) {
      return new SequentialCommandGroup(
          Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "G2E", new PathConstraints(2, 2), true))),
          // Commands.waitSeconds(1), // grabT
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "G2EPart2", new PathConstraints(1, 2), true),
          // Commands.runOnce(poseEstimatorSystem::set180FieldPosition, driveSystem),
          // new PlaceHigh(driveSystem, poseEstimatorSystem, lime, mainArm, shaftEncoder, 190),
          //   Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
          //   new AllLime(driveSystem, poseEstimatorSystem, lime, 0), // april tag?
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem,
                      poseEstimatorSystem,
                      "G2EPart3",
                      new PathConstraints(2, 2),
                      true))),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("I2+1")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(2, 2), true))),
          // grab

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
              driveSystem, poseEstimatorSystem, "I2+1Part3", new PathConstraints(2, 2), true)
          // grab
          );
    }
    if (pathName.equals("I2NoLime")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          // new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  // new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  //  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem,
                      poseEstimatorSystem,
                      "I2+1",
                      new PathConstraints(1.5, 1),
                      true))),
          // Commands.waitSeconds(1), // grab
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1Part2", new PathConstraints(1.5, 1), true),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(1, 1), true),
          //   new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          //   new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
          Commands.waitSeconds(.5));
      //  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5));
    }
    if (pathName.equals("I2")) { // changed pipeline and added grab and move to april
      return new SequentialCommandGroup(
          new ClawEncoderMoveDown(-80, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          new PPIDAutoAim(driveSystem, lime, 44),
          new ChangePipeline(lime),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(1, 1), true))),
          new ClawEncoderMoveDown(-125, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
          Commands.waitSeconds(1),
          Commands.sequence(
              Commands.parallel(
                  new ClawOpenCloseShuffleBoard(25, 5, clawOpenClose), Commands.waitSeconds(1)),
              new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "CUBE"),
              new ClawOpenClose(0, 5, clawOpenClose).withTimeout(2)),
          Commands.runOnce(clawEncoder::posDown, clawEncoder),
          // new PPSwerveFollower(
          //   driveSystem, poseEstimatorSystem, "I2+1MoveApril", new PathConstraints(1, 1), true),
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
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 15, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(2, 2), true))),
          // Commands.waitSeconds(1),
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
          // grab
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "I2+1E", new PathConstraints(2, 2), true),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("C1+E")) {
      return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
          // new PPIDAutoAim(driveSystem, lime, 44),
          new MagicMotionPos(mainArm, 40, 1, 1, .5), // NEW
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "C1+E", new PathConstraints(3, 2), true))),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    if (pathName.equals("G1+E")) {
      return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ClawEncoderMoveDown(-100, clawPivot, clawEncoder, "Cube").withTimeout(1.5),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "move12", new PathConstraints(.5, .5), true)),
          //  new PPIDAutoAim(driveSystem, lime, 44),
          new MagicMotionPos(mainArm, 40, 1, 1, .5), // NEW
          new MagicMotionPos(mainArm, 190, 2.75, 5, .5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 10, 3, 1.5, .5),
                  Commands.waitSeconds(.5),
                  new ClawEncoderMoveUp(0, clawPivot, clawEncoder, "Cube"),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder, 5, 2.5),
                  new PPSwerveFollower(
                      driveSystem, poseEstimatorSystem, "G1+E", new PathConstraints(3, 2), true))),
          new AutoBalance(driveSystem),
          Commands.runOnce(() -> driveSystem.drive(new ChassisSpeeds(0, 0.1, 0)), driveSystem),
          Commands.waitSeconds(0.2),
          Commands.run(driveSystem::stop, driveSystem));
    }
    return null;
  }

  // if (path.equals("StraightNoRotation")) {
  // addCommands(
  // Commands.sequence(
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
  // new MagicMotionPos(mainArm, 210, 0, 0),
  // Commands.waitSeconds(.5),
  // new MagicMotionPos(mainArm, 0, 0, 0),
  // Commands.waitSeconds(.5),
  // new MagicMotionAbsoluteZero(mainArm, shaftEncoder),
  // new PPSwerveFollower(
  // driveSystem,
  // poseEstimatorSystem,
  // "StraightNoRotation",
  // new PathConstraints(2, 2),
  // true),
  // autoBalance));
  // // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // // new PPSwerveFollower(
  // // driveSystem,
  // // poseEstimatorSystem,
  // // "StraightNoRotation",
  // // new PathConstraints(2, 2),
  // // true),
  // // autoBalance);
  // } else if (path.equals("StraightWithRotation")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem,
  // poseEstimatorSystem,
  // "StraightWithRotation",
  // new PathConstraints(1, 1),
  // true));
  // } else if (path.equals("D-F1E")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new MagicMotionPos(mainArm, 210, 0, 0),
  // Commands.waitSeconds(.5),
  // new MagicMotionPos(mainArm, 3, 0, 0),
  // Commands.waitSeconds(.5),
  // new MagicMotionAbsoluteZero(mainArm, shaftEncoder),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "D-F1E", new PathConstraints(2, 2), true),
  // autoBalance);
  // } else if (path.equals("A2E")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "A2E", new PathConstraints(1, 1), true));
  // } else if (path.equals("D1+1")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "D1+1", new PathConstraints(1, 1), true));
  // } else if (path.equals("F1+1")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "F1+1", new PathConstraints(1, 1), true));
  // } else if (path.equals("G2E")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "G2E", new PathConstraints(1, 1), true));
  // } else if (path.equals("I2+1")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(1, 1), true));
  // } else if (path.equals("I2+1E")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "I2+1E", new PathConstraints(1, 1), true));
  // } else if (path.equals("I3")) {
  // addCommands(
  // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  // new PPSwerveFollower(
  // driveSystem, poseEstimatorSystem, "I3", new PathConstraints(1, 1), true));
  // }
  // }
}
