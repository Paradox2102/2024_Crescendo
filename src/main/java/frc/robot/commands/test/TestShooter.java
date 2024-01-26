// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooter extends Command {
  ShooterSubsystem m_subsystem;
  HolderSubsystem m_holderSubsystem;
  boolean m_shoot;
  /** Creates a new TestShooter. */
  public TestShooter(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem, boolean shoot) {
    m_subsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_shoot = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_shoot) {
      m_subsystem.setVelocityRPM(-2100); // -1750
      m_holderSubsystem.setVelocityRPM(1000);
    } else {
      m_subsystem.setVelocityRPM(2000);
      m_holderSubsystem.setVelocityRPM(-500);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    m_holderSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
