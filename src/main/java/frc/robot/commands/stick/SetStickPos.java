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

  public SetStickPos(StickSubsystem stickSubsystem) {
    m_subsystem = stickSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_runningShootSequence && Constants.States.m_speakerMode) {
      m_power = 0;
    } else {
      m_power = m_subsystem.m_retracted ? 1 : -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setPower(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    m_subsystem.m_retracted = !m_subsystem.m_retracted;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double pos = m_subsystem.getPositionInRotations();
    return m_subsystem.m_retracted ?  pos >= Constants.StickConstants.k_maxExtentRotations : pos <= 0;
  }
}
