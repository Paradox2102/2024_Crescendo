// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class D2Intake extends Command {
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;
  boolean m_intake;
  /** Creates a new D2Intake. */
  public D2Intake(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem, boolean intake) {
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake) {
      m_shooterSubsystem.setVelocityRPM(2000);
      m_holderSubsystem.setVelocityRPM(500);
    } else {
      m_shooterSubsystem.setVelocityRPM(-2000);
      m_holderSubsystem.setVelocityRPM(-500);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_holderSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
