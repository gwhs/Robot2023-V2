// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import frc.robot.Constants.DrivetrainConstants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

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

}
