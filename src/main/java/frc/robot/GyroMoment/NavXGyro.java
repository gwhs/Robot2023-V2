// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class NavXGyro extends AbstractGyro {

    private AHRS ahrs;

    public NavXGyro()
    {
        ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);
    }

    @Override
    public void reset()
    {
        ahrs.reset();
    }

    @Override
    public void calibrate()
    {
        ahrs.calibrate();
    }

    @Override
    public double getAngle()
    {
        return ahrs.getAngle();
    }

    @Override
    public double getYaw()
    {
        return ahrs.getYaw();
    }

    @Override
    public double getPitch()
    {
        return ahrs.getPitch();
    }

    @Override
    public double getRoll()
    {
        return ahrs.getRoll();
    }

    @Override
    public void configMountPoseRoll(double roll)
    {
        //ignored
    }

    @Override
    public void configMountPoseYaw(double yaw)
    {
        //ignored
    }

    @Override
    public Rotation2d getRotation2d()
    {
        return ahrs.getRotation2d();
    }

    @Override
    public void setYaw(double yaw)
    {
        //ignored
    }

    @Override
    public double getYawRate()
    {
        return ahrs.getRawGyroY();
    }

    @Override
    public double getPitchRate()
    {
        return ahrs.getRawGyroX();
    }

    @Override
    public double getRollRate()
    {
        return ahrs.getRawGyroZ();
    }
}
