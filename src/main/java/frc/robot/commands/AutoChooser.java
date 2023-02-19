// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoChooser extends CommandBase {

  private double autoSelection;
  private GenericEntry pathNumber;

  // SendableChooser<Integer> pathChooser = new SendableChooser<>();

  SendableChooser<String> m_chooser = new SendableChooser<>();

  /** Creates a new AutoChooser. */
  public AutoChooser() {

    final ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    // pathChooser.addDefault("Path 1", 1)

    m_chooser.setDefaultOption("Path 1", "Autonomous 1");
    m_chooser.addOption("Path 2", "Autonomous 2");
    m_chooser.addOption("Path 3", "Autonomous 3");

    tab.add(m_chooser);

  }

  @Override
  public void initialize() {
    // if auto path input from shuffleboard is one, print works

    
    System.out.println(m_chooser.getSelected());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
