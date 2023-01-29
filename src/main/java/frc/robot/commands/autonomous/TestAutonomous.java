// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.List;

public class TestAutonomous extends SequentialCommandGroup {
  /** Creates a new TestAutonomous. */
  public TestAutonomous(
      DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimatorSystem) {

    List<PathPlannerTrajectory> trajectories =
        PathPlanner.loadPathGroup("8 go straight", new PathConstraints(0.5, 0.5));
    PathPlannerTrajectory path = PathPlanner.loadPath("8 go straight", 0.5, 0.5);

    PPSwerveControllerCommand drive =
    //     new PPSwerveControllerCommand(
    //         path,
    //         poseEstimatorSystem::getCurrentPose,
    //         DrivetrainConstants.KINEMATICS,
    //         new PIDController(1, 0, 0),
    //         new PIDController(1, 0, 0),
    //         drivetrain.getThetaController(),
    //         drivetrain::setModuleStates,
    //         drivetrain);
    DrivetrainSubsystem.followTrajectory(drivetrain, poseEstimatorSystem, trajectories.get(0));

    addCommands(drive, 
    new InstantCommand(() -> drivetrain.drive(new ChassisSpeeds(0, 0 ,0))));
  }
}
