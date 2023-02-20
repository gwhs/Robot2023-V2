package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.PPSwerveFollower;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class TestAutoCommands extends SequentialCommandGroup {

  private DrivetrainSubsystem driveSystem;
  private PoseEstimatorSubsystem poseEstimatorSystem;
  private MagicMotion mainArm;
  private BoreEncoder shaftEncoder;
  private AutoBalance autoBalance;

  public TestAutoCommands(
      DrivetrainSubsystem d,
      PoseEstimatorSubsystem poseEstimatorSystem,
      MagicMotion m,
      BoreEncoder b,
      AutoBalance autoBalance,
      String path) {
    this.driveSystem = d;
    this.poseEstimatorSystem = poseEstimatorSystem;
    this.mainArm = m;
    this.shaftEncoder = b;
    this.autoBalance = autoBalance;

    if (path.equals("StraightNoRotation")) {
      addCommands(
          new PPSwerveFollower(
              driveSystem,
              poseEstimatorSystem,
              "StraightNoRotation",
              new PathConstraints(1, 1),
              true));
    } else if (path.equals("StraightWithRotation")) {
      addCommands(
          new PPSwerveFollower(
              driveSystem,
              poseEstimatorSystem,
              "StraightWithRotation",
              new PathConstraints(1, 1),
              true));
    } else if (path.equals("D-F1E")) {
      addCommands(
          new MagicMotionPos(mainArm, 210, 0, 0),
          Commands.waitSeconds(.5),
          new MagicMotionPos(mainArm, 10, 0, 0),
          Commands.waitSeconds(.5),
          new MagicMotionAbsoluteZero(mainArm, shaftEncoder),
          new PPSwerveFollower(
              driveSystem, poseEstimatorSystem, "D-F1E", new PathConstraints(1, 1), true),
          autoBalance);
    } else if (path.equals("A2E")) {

    } else if (path.equals("D1+1")) {

    } else if (path.equals("F1+1")) {

    } else if (path.equals("G2E")) {

    } else if (path.equals("I2+1")) {

    } else if (path.equals("I2+1E")) {

    } else if (path.equals("I3")) {

    }
  }
}
