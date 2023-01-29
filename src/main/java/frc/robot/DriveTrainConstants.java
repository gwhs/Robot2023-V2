// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.lang.Math.toRadians;

/** Steer offsets, motor ids, encoder ids for all modules */
public class DriveTrainConstants {
  public int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  public int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
  public int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
  public int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
  public int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
  public int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  public int BACK_LEFT_MODULE_STEER_ENCODER = 12;
  public int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
  public int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
  public int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
  public double FRONT_LEFT_MODULE_STEER_OFFSET;
  public double FRONT_RIGHT_MODULE_STEER_OFFSET;
  public double BACK_LEFT_MODULE_STEER_OFFSET;
  public double BACK_RIGHT_MODULE_STEER_OFFSET;

  public static DriveTrainConstants spring = new DriveTrainConstants(52, 126.8, 134, 341.1);

  // hana drivemotors are inverted, steer motors are not
  public static DriveTrainConstants hana = new DriveTrainConstants(336.6, 321.05, 75.9, 358.5);

  public DriveTrainConstants(
      double FRONT_LEFT_MODULE_STEER_OFFSET,
      double FRONT_RIGHT_MODULE_STEER_OFFSET,
      double BACK_LEFT_MODULE_STEER_OFFSET,
      double BACK_RIGHT_MODULE_STEER_OFFSET) {
    this.FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(FRONT_LEFT_MODULE_STEER_OFFSET);
    this.FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(FRONT_RIGHT_MODULE_STEER_OFFSET);
    this.BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(BACK_LEFT_MODULE_STEER_OFFSET);
    this.BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(BACK_RIGHT_MODULE_STEER_OFFSET);
  }
}
