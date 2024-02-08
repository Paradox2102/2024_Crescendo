// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * RESPONSIBLILIES --->   ToggleElevatorPosition(start high ---> low)normal command start 20 inches
 *  (constants- max instant -20, min extent -0); use setPosition
 * 
 * TBD ---> manual command - (Reference:phonix elevator)
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleElevatorPosition extends Command {
    ElevatorSubsystem m_subsystem;
    private double m_position;

  /** Creates a new ToggleElevatorPosition. */
  public ToggleElevatorPosition(ElevatorSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    m_position = position;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPosition(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
