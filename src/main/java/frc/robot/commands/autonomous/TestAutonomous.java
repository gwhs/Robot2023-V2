// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.List;

public class TestAutonomous extends SequentialCommandGroup {
  /** Creates a new TestAutonomous. */
  public TestAutonomous(
      DrivetrainSubsystem driveSystem, PoseEstimatorSubsystem poseEstimatorSystem) {

    List<PathPlannerTrajectory> trajectories =
        PathPlanner.loadPathGroup("8 go straight", new PathConstraints(3, 2));
    PathPlannerTrajectory path = PathPlanner.loadPath("8 go straight", 3, 2);

    PPSwerveControllerCommand drive =
        TrajectoryCommands.followTrajectory(poseEstimatorSystem, trajectories.get(0), driveSystem);

    addCommands(drive);
  }
}
