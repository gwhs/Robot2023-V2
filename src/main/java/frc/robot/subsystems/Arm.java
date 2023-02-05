// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX testTalon;
  /** Creates a new Arm. */
  public Arm(int id) {
    testTalon = new TalonFX(id);
  }

  public void setPercent(double speed) {
    testTalon.set(ControlMode.PercentOutput, speed);
  }
}
