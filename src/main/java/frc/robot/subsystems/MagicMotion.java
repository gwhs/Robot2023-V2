// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import java.lang.reflect.GenericArrayType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MagicMotion extends SubsystemBase {
  TalonFX testTalon;
  /** Creates a new MagicMotion. */
  public MagicMotion(int id) {
    testTalon = new TalonFX(id);
    resetPosition();
  }

	public void robotInit() {
		/* Factory default hardware to prevent unexpected behavior */
		testTalon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		testTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.Arm.kPIDLoopIdx,
				Constants.Arm.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		testTalon.configNeutralDeadband(0.001, Constants.Arm.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		testTalon.setSensorPhase(false);
		testTalon.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // testTalon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		testTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Arm.kTimeoutMs);
		testTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Arm.kTimeoutMs);

		/* Set the peak and nominal outputs */
		testTalon.configNominalOutputForward(0, Constants.Arm.kTimeoutMs);
		testTalon.configNominalOutputReverse(0, Constants.Arm.kTimeoutMs);
		testTalon.configPeakOutputForward(1, Constants.Arm.kTimeoutMs);
		testTalon.configPeakOutputReverse(-1, Constants.Arm.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		testTalon.selectProfileSlot(Constants.Arm.kSlotIdx, Constants.Arm.kPIDLoopIdx);
		testTalon.config_kF(Constants.Arm.kSlotIdx, Constants.Arm.kGains.kF, Constants.Arm.kTimeoutMs);
		testTalon.config_kP(Constants.Arm.kSlotIdx, Constants.Arm.kGains.kP, Constants.Arm.kTimeoutMs);
		testTalon.config_kI(Constants.Arm.kSlotIdx, Constants.Arm.kGains.kI, Constants.Arm.kTimeoutMs);
		testTalon.config_kD(Constants.Arm.kSlotIdx, Constants.Arm.kGains.kD, Constants.Arm.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */


		/* Zero the sensor once on robot boot up */
		testTalon.setSelectedSensorPosition(0, Constants.Arm.kPIDLoopIdx, Constants.Arm.kTimeoutMs);
	}

  public void setAng(double angle, double vel, double accel){
    //double velocity = vel / 360 * Constants.FALCON_TICKS * 20;
    //double acceleration = accel / 360 * Constants.FALCON_TICKS * 20;

    //System.out.println("Velocity: " + velocity);
    //System.out.println("Acceleration: " + acceleration);

    testTalon.configMotionCruiseVelocity(10000, Constants.Arm.kTimeoutMs);
		testTalon.configMotionAcceleration(10000, Constants.Arm.kTimeoutMs);

    testTalon.set(ControlMode.MotionMagic, angle * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO);
  }

  public void resetPosition(){
    testTalon.setSelectedSensorPosition(0);
  }

  public double getAng(){
    return testTalon.getSelectedSensorPosition();
  }
}
