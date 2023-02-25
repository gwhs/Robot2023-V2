package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.function.Supplier;

public class TrajectoryCommands {
  public static PPSwerveControllerCommand followTrajectory(
      PoseEstimatorSubsystem s, PathPlannerTrajectory traj, DrivetrainSubsystem drivetrainsubsystem) {
    return new PPSwerveControllerCommand(
        traj,
        s::getCurrentPose,
        Constants.DrivetrainConstants.KINEMATICS,
        Constants.AutoConstants.m_translationController,
        Constants.AutoConstants.m_strafeController,
        Constants.AutoConstants.m_thetaController,
        drivetrainsubsystem::setModuleStates,
        drivetrainsubsystem);
  }
  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   *
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public static Command createCommandForTrajectory(
      Trajectory trajectory, Supplier<Pose2d> poseSupplier, DrivetrainSubsystem drivetrainsubsystem) {
    var thetaController =
        new ProfiledPIDController(
            -AutoConstants.THETA_kP,
            AutoConstants.THETA_kI,
            AutoConstants.THETA_kD,
            AutoConstants.THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            poseSupplier,
            DrivetrainConstants.KINEMATICS,
            new PIDController(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
            new PIDController(AutoConstants.Y_kP, AutoConstants.Y_kI, AutoConstants.Y_kD),
            thetaController,
            drivetrainsubsystem::setModuleStates,
            drivetrainsubsystem);

    return swerveControllerCommand;
  }
}
