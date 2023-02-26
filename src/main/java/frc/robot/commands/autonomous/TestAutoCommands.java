package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.PPSwerveFollower;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.commands.AutoBalanceFast;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public final class TestAutoCommands {

  private DrivetrainSubsystem driveSystem;
  private PoseEstimatorSubsystem poseEstimatorSystem;
  private MagicMotion mainArm;
  private BoreEncoder shaftEncoder;
  private String pathName;

  public TestAutoCommands(
      DrivetrainSubsystem d,
      PoseEstimatorSubsystem poseEstimatorSystem,
      MagicMotion m,
      BoreEncoder b,
      String p) {
    this.driveSystem = d;
    this.poseEstimatorSystem = poseEstimatorSystem;
    this.mainArm = m;
    this.shaftEncoder = b;
    this.pathName = p;
  }

  public SequentialCommandGroup getAutoCommand() {
    if (pathName.equals("HajelPath")) {
      return new SequentialCommandGroup(
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
          new MagicMotionPos(mainArm, 210, 0, 0),
          Commands.waitSeconds(.5),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new MagicMotionPos(mainArm, 0, 0, 0),
                  Commands.waitSeconds(.5),
                  new MagicMotionAbsoluteZero(mainArm, shaftEncoder)),
              new PPSwerveFollower(
                  driveSystem, poseEstimatorSystem, "HajelPath", new PathConstraints(2, 2), true)),
          new AutoBalanceFast(driveSystem));
    }
    return null;
  }

  //   if (path.equals("StraightNoRotation")) {
  //     addCommands(
  //         Commands.sequence(
  //             new PPSwerveFollower(
  //                 driveSystem, poseEstimatorSystem, "move12", new PathConstraints(2, 2), true),
  //             new MagicMotionPos(mainArm, 210, 0, 0),
  //             Commands.waitSeconds(.5),
  //             new MagicMotionPos(mainArm, 0, 0, 0),
  //             Commands.waitSeconds(.5),
  //             new MagicMotionAbsoluteZero(mainArm, shaftEncoder),
  //             new PPSwerveFollower(
  //                 driveSystem,
  //                 poseEstimatorSystem,
  //                 "StraightNoRotation",
  //                 new PathConstraints(2, 2),
  //                 true),
  //             autoBalance));
  //     // Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //     // new PPSwerveFollower(
  //     //     driveSystem,
  //     //     poseEstimatorSystem,
  //     //     "StraightNoRotation",
  //     //     new PathConstraints(2, 2),
  //     //     true),
  //     // autoBalance);
  //   } else if (path.equals("StraightWithRotation")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem,
  //             poseEstimatorSystem,
  //             "StraightWithRotation",
  //             new PathConstraints(1, 1),
  //             true));
  //   } else if (path.equals("D-F1E")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new MagicMotionPos(mainArm, 210, 0, 0),
  //         Commands.waitSeconds(.5),
  //         new MagicMotionPos(mainArm, 3, 0, 0),
  //         Commands.waitSeconds(.5),
  //         new MagicMotionAbsoluteZero(mainArm, shaftEncoder),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "D-F1E", new PathConstraints(2, 2), true),
  //         autoBalance);
  //   } else if (path.equals("A2E")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "A2E", new PathConstraints(1, 1), true));
  //   } else if (path.equals("D1+1")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "D1+1", new PathConstraints(1, 1), true));
  //   } else if (path.equals("F1+1")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "F1+1", new PathConstraints(1, 1), true));
  //   } else if (path.equals("G2E")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "G2E", new PathConstraints(1, 1), true));
  //   } else if (path.equals("I2+1")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "I2+1", new PathConstraints(1, 1), true));
  //   } else if (path.equals("I2+1E")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "I2+1E", new PathConstraints(1, 1), true));
  //   } else if (path.equals("I3")) {
  //     addCommands(
  //         Commands.runOnce(poseEstimatorSystem::resetFieldPosition, driveSystem),
  //         new PPSwerveFollower(
  //             driveSystem, poseEstimatorSystem, "I3", new PathConstraints(1, 1), true));
  //   }
  // }
}
