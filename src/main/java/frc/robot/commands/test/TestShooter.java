// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooter extends Command {
  ShooterSubsystem m_shooterSubsystem;
  HolderSubsystem m_holderSubsystem;
  PivotSubsystem m_pivotSubsystem;
  boolean m_shoot;
  /** Creates a new TestShooter. */
  public TestShooter(ShooterSubsystem shooterSubsystem, HolderSubsystem holderSubsystem, PivotSubsystem pivotSubsystem, boolean shoot) {
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_shoot = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_shoot) {
      if (Constants.k_speaker){
        m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_speakerVelocityRPM);
      } else {
        m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_ampVelocityRPM);
      }
    } else {
      m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_intakeVelocityRPM);
      m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_intakeVelocityRPM);
      m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shoot) {
      if (Constants.k_speaker && Math.abs(m_shooterSubsystem.getVelocityRPM() - Constants.ShooterConstants.k_speakerVelocityRPM) < 25) {
        m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_speakerVelocityRPM);
      } else if (Math.abs(m_shooterSubsystem.getVelocityRPM() - Constants.ShooterConstants.k_ampVelocityRPM) < 25) {
        m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_ampVelocityRPM);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_holderSubsystem.stop();
    m_pivotSubsystem.setPositionDegrees(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
