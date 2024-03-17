// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stick;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.StickSubsystem;

public class SetStickPos extends Command {
  /** Creates a new SetStickPos. */
  StickSubsystem m_subsystem;
  double m_power = 1;
  boolean m_extending = true;
  boolean m_runningShootSequence;

  public SetStickPos(StickSubsystem stickSubsystem, boolean runningShootSequence) {
    m_subsystem = stickSubsystem;
    m_runningShootSequence = runningShootSequence;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_runningShootSequence && Constants.States.m_speakerMode) {}
    else {
      m_subsystem.setPose(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPose(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
