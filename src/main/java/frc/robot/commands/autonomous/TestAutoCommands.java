package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.PPSwerveFollower;
import frc.robot.commands.Arm.MagicMotionAbsoluteZero;
import frc.robot.commands.Arm.MagicMotionPos;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class TestAutoCommands extends SequentialCommandGroup  {

  private DrivetrainSubsystem driveSystem;
  private PoseEstimatorSubsystem poseEstimatorSystem;
  private MagicMotion mainArm;
  private BoreEncoder shaftEncoder;

  public TestAutoCommands(DrivetrainSubsystem d,PoseEstimatorSubsystem poseEstimatorSystem,MagicMotion m,BoreEncoder b) {
    this.driveSystem = d;
    this.poseEstimatorSystem = poseEstimatorSystem;
    this.mainArm = m;
    this.shaftEncoder = b;

    addCommands(
        new PPSwerveFollower(
            driveSystem,
            poseEstimatorSystem,
            "StraightWithRotation",
            new PathConstraints(1, 1),
            true),

    new MagicMotionPos(mainArm, 210, 0, 0),
    Commands.waitSeconds(.5),
    new MagicMotionPos(mainArm, 0, 0, 0),
    Commands.waitSeconds(.5),
    new MagicMotionAbsoluteZero(mainArm, shaftEncoder)
    
    );
  }

}
