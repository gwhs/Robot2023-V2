// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

/** Add your docs here. */
public abstract class AbstractGyro {

    public abstract void reset();
    public abstract void calibrate();
    public abstract double getAngle();
    public abstract double getYaw();
    public abstract double getPitch();
    public abstract double getRoll();

}
