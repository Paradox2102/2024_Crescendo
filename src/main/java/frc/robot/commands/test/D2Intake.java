// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class D2Intake extends Command {
  ManipulatorSubsystem m_frontSubsystem;
  ManipulatorSubsystem m_backSubsystem;
  boolean m_intake;
  /** Creates a new D2Intake. */
  public D2Intake(ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem, boolean intake) {
    m_frontSubsystem = frontSubsystem;
    m_backSubsystem = backSubsystem;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontSubsystem, m_backSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_frontSubsystem.setVelocityRPM(2000);
    m_backSubsystem.setVelocityRPM(500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_frontSubsystem.stop();
    m_backSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.States.m_hasGamePiece;
  }
}
