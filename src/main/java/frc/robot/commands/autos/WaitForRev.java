// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitForRev extends Command {
  /** Creates a new WaitForRev. */
  HolderSubsystem m_holderSubsytem;
  ShooterSubsystem m_shooterSubsystem;
  public WaitForRev(HolderSubsystem holderSubsystem, ShooterSubsystem shooterSubsystem) {
    m_holderSubsytem = holderSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.States.m_isCompetition ? m_holderSubsytem.isReady() : m_shooterSubsystem.isReady();
  }
}
