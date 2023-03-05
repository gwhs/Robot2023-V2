// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new claw. */
  private CANSparkMax m_motor;

  private SparkMaxPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private RelativeEncoder m_encoder;

  public Claw(int id, CANSparkMax.MotorType type) {
    m_motor = new CANSparkMax(id, type);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(false);
    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object is
     * constructed by calling the getPIDController() method on an existing CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    /**
     * By default, the PID controller will use the Hall sensor from a NEO for its feedback device.
     * Instead, we can set the feedback device to the alternate encoder object
     */
    m_pidController.setFeedbackDevice(m_encoder);

    /**
     * From here on out, code looks exactly like running PID control with the built-in NEO encoder,
     * but feedback will come from the alternate encoder
     */

    // PID coefficients
    kP = 0.1;
    kI = 0;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  public void setPercent(double percent) {
    m_motor.set(percent);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public void resetPosition() {
    m_encoder.setPosition(0);
  }

  public void setPosition(double angle) {
    m_pidController.setReference(
        angle / 360.0 * Constants.Claw.GEAR_RATIO, CANSparkMax.ControlType.kPosition);
  }

  public void setLimit(int num) {
    m_motor.setSmartCurrentLimit(num);
  }

  public void brake() {
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {}
}
