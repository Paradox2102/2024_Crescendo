// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamePieceManipulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakeCommand extends Command {
  ManipulatorSubsystem m_holderSubsystem;
  ManipulatorSubsystem m_shooterSubsystem;
  PivotSubsystem m_pivotSubsystem;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(ManipulatorSubsystem holderSubsystem, ManipulatorSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_holderSubsystem = holderSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    addRequirements(m_holderSubsystem, m_shooterSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
