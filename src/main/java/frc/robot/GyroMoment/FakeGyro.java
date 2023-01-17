// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class FakeGyro extends AbstractGyro {

    private double angle;
    private double yaw;
    private double pitch;
    private double roll;

    public FakeGyro()
    {
        angle = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;
    }

    @Override
    public void reset()
    {
        angle = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;
    }

    @Override
    public void calibrate()
    {
        //ignored
    }

    @Override
    public double getAngle()
    {
        return angle;
    }

    @Override
    public double getYaw()
    {
        return yaw;
    }

    @Override
    public double getPitch()
    {
        return pitch;
    }

    @Override
    public double getRoll()
    {
        return roll;
    }

    public void setAngle(double angle)
    {
        this.angle = angle;
    }

    public void setYaw(double yaw)
    {
        this.yaw = yaw;
    }

    public void setPitch(double pitch)
    {
        this.pitch = pitch;
    }

    public void setRoll(double roll)
    {
        this.roll = roll;
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
        return new Rotation2d(Math.toRadians(yaw));
    }

}
