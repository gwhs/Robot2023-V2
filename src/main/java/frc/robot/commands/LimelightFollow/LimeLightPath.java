package frc.robot.commands.LimeLightPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;


public class LimeLightPath extends CommandBase{

    private LimeLightSub LimeLight = new LimelightSub("");
    private DrivetrainSubsystem drivetrainSub;
    private PoseEstimatorSystem poseEstimatorSubsystem;

    public LimeLightSub(
     LimeLightSub limeLightSub,
     DrivetrainSubsytem drivetrainSub,
     PoseEstimatorSubsystem poseEstimatorSubsystem,) {
        this.limeLight = LimeLightSub;
        this.DrivetrainSub = DrivetrainSub;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        
        addRequirements(DrivetrainSub, limeLight);


        
    }




    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {}

}



