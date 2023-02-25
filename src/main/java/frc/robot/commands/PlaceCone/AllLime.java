// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.PlaceCone;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// import frc.robot.Constants.LimeLightConstants;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.LimeVision.LimeLightSub;
// import frc.robot.subsystems.PoseEstimatorSubsystem;
// import frc.robot.commands.Lime.PPIDAutoAim;
// import frc.robot.commands.Lime.Rotate;



// public class AllLime extends SequentialCommandGroup {
//   /** Creates a new AllLime. */
//   public AllLime(
//       DrivetrainSubsystem drivetrainSubsystem,
//       PoseEstimatorSubsystem poseEstimatorSubsystem,
//       LimeLightSub limeLightSub) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//         new PPIDAutoAim(
//             drivetrainSubsystem,
//             poseEstimatorSubsystem,
//             limeLightSub),
//         new WaitCommand(.5),
//         new Rotate(drivetrainSubsystem, poseEstimatorSubsystem, 0),
//         new StraightWheel(drivetrainSubsystem),
//         new Sideways(drivetrainSubsystem, poseEstimatorSubsystem, limeLightSub),
//         new StraightWheel(drivetrainSubsystem),
//         new PPIDAutoAim(
//             drivetrainSubsystem,
//             poseEstimatorSubsystem,
//             limeLightSub,
//             LimeLightConstants.UPPER_DISTANCE_SHOOT));
//   }
// }
