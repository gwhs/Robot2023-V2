// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimeLightConstants;

public class LimeLightSub extends SubsystemBase {

  // set up a new instance of NetworkTables (the api/library used to read values from limelight)
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

  // return network table values for tx and ty using getEntry()
  NetworkTableEntry tv =
      networkTable.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
  NetworkTableEntry tx =
      networkTable.getEntry(
          "tx"); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees |
  // LL2: -29.8 to 29.8 degrees)
  NetworkTableEntry ty =
      networkTable.getEntry(
          "ty"); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees |
  // LL2: -24.85 to 24.85 degrees)
  NetworkTableEntry ta = networkTable.getEntry("ta"); // Target Area (0% of image to 100% of image)
  NetworkTableEntry ts = networkTable.getEntry("ts"); // Skew or rotation (-90 degrees to 0 degrees)

  private double kCameraHeight =
      LimeLightConstants.CAMERA_HEIGHT; // LimelightConstants.kCameraHeight;
  private double kTargetHeight =
      LimeLightConstants.TARGET_HEIGHT; // LimelightConstants.kTargetHeight;

  private double theta;
  private double distance;

  private LimeLightComms limelight_comm;

  /** Creates a new LimeLightSub. */
  public LimeLightSub(String limelight_networktable_name) {
    limelight_comm = new LimeLightComms(limelight_networktable_name);
    limelight_comm.set_entry_double("ledMode", 3);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    SmartDashboard.putNumber("theta", getTheta());
    SmartDashboard.putNumber("X-Distance", getXDistance());
    SmartDashboard.putNumber("Y-Distance", getYDistance());
    SmartDashboard.putNumber("AngleToTarget", getAngle());
    System.out.println(DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    // This method will be called once per scheduler run
  }

  public double getTv() {
    return tv.getDouble(0);
  }

  public double getTx() {
    return tx.getDouble(0);
  }

  public double getTy() {
    return ty.getDouble(0);
  }

  public double getTheta() {
    return getTy() + LimeLightConstants.MOUNTING_ANGLE;
  }

  public double getXDistance() {
    return (kTargetHeight - kCameraHeight)
        / (Math.tan(Math.toRadians(getTy() + LimeLightConstants.MOUNTING_ANGLE)));
  }

  public double getYDistance() {
    return Math.tan(Math.toRadians(getTx())) * getXDistance();
  }

  public double getAngle() {
    return Math.toRadians(-getTx());
  }

  public double[] chassisValuesLower() {
    /*
    [1,2,3]
    1 is x velocity
    2 is y velocity
    3 is degrees rotation
    get the angle using atan2, it returns radians
    use sin and cos to get values to reach max speed
    not really sure about the angle yet.
    */
    double distanceError = getXDistance() - LimeLightConstants.LOWER_DISTANCE_SHOOT;
    double[] x = new double[3];
    x[0] = distanceError;
    x[1] = 0;
    x[2] = -getTx();
    // getAngle()
    //     / (((Math.sqrt(x[0] * x[0]) + x[1] * x[1]))
    //         / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    return x;
  }
}
