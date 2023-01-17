// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import frc.robot.Constants.DrivetrainConstants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class PigeonGyro extends AbstractGyro {

    private WPI_Pigeon2 pigeon;

    public PigeonGyro()
    {
        pigeon = new WPI_Pigeon2(DrivetrainConstants.PIGEON_ID);
    }

    @Override
    public void reset()
    {
        pigeon.reset();
    }

    @Override
    public void calibrate()
    {
        pigeon.calibrate();
    }

    @Override
    public double getAngle()
    {
        return pigeon.getAngle();
    }

    @Override
    public double getYaw()
    {
        return pigeon.getYaw();
    }

    @Override
    public double getPitch()
    {
        return pigeon.getPitch();
    }

    @Override
    public double getRoll()
    {
        return pigeon.getRoll();
    }

    @Override
    public void configMountPoseRoll(double roll)
    {
        pigeon.configMountPoseRoll(roll);
    }

    @Override
    public void configMountPoseYaw(double yaw)
    {
        pigeon.configMountPoseYaw(yaw);
    }

    @Override
    public Rotation2d getRotation2d()
    {
        return pigeon.getRotation2d();
    }

    @Override 
    public void setYaw(double yaw)
    {
        pigeon.setYaw(yaw);
    }
    
    @Override
    public double getYawRate()
    {
        double[] xyz_dps = new double[3];
        pigeon.getRawGyro(xyz_dps);
        return xyz_dps[1];
    }

    @Override
    public double getPitchRate()
    {
        double[] xyz_dps = new double[3];
        pigeon.getRawGyro(xyz_dps);
        return xyz_dps[0];
    }

    @Override
    public double getRollRate()
    {
        double[] xyz_dps = new double[3];
        pigeon.getRawGyro(xyz_dps);
        return xyz_dps[2];
    }

}
