// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceCone;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimeVision.LimeLightSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangePipeline extends InstantCommand {
  private LimeLightSub limelightsub;

  public ChangePipeline(LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelightsub = limeLightSub;
    addRequirements(limelightsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelightsub.getPipeline() > .5) {
      limelightsub.setPipeline(0);
    } else {
      limelightsub.setPipeline(1);
    }
    System.out.println(limelightsub.getPipeline());
  }
}
