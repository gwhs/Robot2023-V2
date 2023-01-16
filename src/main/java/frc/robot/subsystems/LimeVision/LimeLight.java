// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Copied from 
 * https://github.com/FRC6854/VIKING/blob/87d32f06d99d272e8859735a67003e81cb4eae8b/src/main/java/viking/vision/limelight/LimelightComm.java
 * 
 */

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight{
  private String networktable_name;

  public LimeLight(String limelight_networktable_name) {
		networktable_name = limelight_networktable_name;
	}

	public double get_entry_double(String entry_name) {
		return NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.getDouble(0);
	}

	public Number get_entry_number(String entry_name) {
		return NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.getNumber(0);
	}

	public double[] get_entry_double_array(String entry_name) {
		return NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.getDoubleArray(new double[0]);
	}

	public Number[] get_entry_number_array(String entry_name) {
		return NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.getNumberArray(new Number[0]);
	}

	public void set_entry_double(String entry_name, double value) {
		NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.setDouble(value);
	}

	public void set_entry_number(String entry_name, Number value) {
		NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.setNumber(value);
	}

	public void set_entry_double_array(String entry_name, double[] array) {
		NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.setDoubleArray(array);
	}

	public void set_entry_number_array(String entry_name, Number[] array) {
		NetworkTableInstance.getDefault()
			.getTable(networktable_name)
			.getEntry(entry_name)
			.getNumberArray(new Number[0]);
	}
}





	