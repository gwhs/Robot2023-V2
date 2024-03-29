// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotSetup;
import java.util.ArrayList;
import java.util.List;
// import frc.lib2539.logging.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotSetup robot = Constants.ryker;

  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    // Disable default NetworkTables logging
    // DataLogManager.logNetworkTables(false);

    // Begin controller inputs
    // if (isReal()) {
    //   DriverStation.startDataLog(DataLogManager.getLog());
    // }

    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("Robot", robot.name());

    String logfolder = "/home/lvuser";
    logger.addDataReceiver(new WPILOGWriter(logfolder));
    logger.addDataReceiver(new NT4Publisher());

    switch (robot.name()) {
      case "hana":
        LoggedPowerDistribution.getInstance(0, ModuleType.kCTRE);
        break;
      case "chris":
        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        break;
      case "calliope":
        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        break;
      case "ryker":
        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        break;
      default:
        LoggedPowerDistribution.getInstance(0, ModuleType.kCTRE);
        break;
    }

    setUseTiming(true);
    logger.start();

    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Log list of NT clients
    List<String> clientNames = new ArrayList<>();
    List<String> clientAddresses = new ArrayList<>();
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      clientNames.add(client.remote_id);
      clientAddresses.add(client.remote_ip);
    }
    Logger.getInstance()
        .recordOutput("NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
    Logger.getInstance()
        .recordOutput(
            "NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));

    // Logger.log("/Robot/Battery Voltage", RobotController.getBatteryVoltage());
    // Logger.update();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
