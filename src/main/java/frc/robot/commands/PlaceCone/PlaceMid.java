// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMid extends SequentialCommandGroup {
  /** Creates a new PlaceMid. */
  public PlaceMid(DrivetrainSubsystem drivetrainSubsystem, LimeLightSub limeLightSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PPIDAutoAim(drivetrainSubsystem, limeLightSub, LimeLightConstants.LOWER_DISTANCE_SHOOT)
    //missing shoot command
    );
  }
}
