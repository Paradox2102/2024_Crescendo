// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TestShooter extends Command {
  ManipulatorSubsystem m_frontSubsystem;
  ManipulatorSubsystem m_backSubsystem;
  PivotSubsystem m_pivotSubsystem;
  boolean m_shoot;
  /** Creates a new TestShooter. */
  public TestShooter(ManipulatorSubsystem frontSubsystem, ManipulatorSubsystem backSubsystem, PivotSubsystem pivotSubsystem, boolean shoot) {
    m_frontSubsystem = frontSubsystem;
    m_backSubsystem = backSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_shoot = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontSubsystem, m_backSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_shoot) {
      if (Constants.States.m_speakerMode){
        m_frontSubsystem.setVelocityRPM(Constants.FrontConstants.k_speakerShootVelocityRPM);
      } else {
        m_frontSubsystem.setVelocityRPM(Constants.FrontConstants.k_ampShootVelocityRPM);
      }
    } else {
      m_frontSubsystem.setVelocityRPM(Constants.FrontConstants.k_intakeVelocityRPM);
      m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_intakeVelocityRPM);
      m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shoot) {
      if (Constants.States.m_speakerMode && Math.abs(m_frontSubsystem.getVelocityRPM() - Constants.FrontConstants.k_speakerShootVelocityRPM) < Constants.FrontConstants.k_deadzone) {
        m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_speakerFeedPower);
      } else if (Math.abs(m_frontSubsystem.getVelocityRPM() - Constants.FrontConstants.k_ampShootVelocityRPM) < Constants.FrontConstants.k_deadzone) {
        m_backSubsystem.setVelocityRPM(Constants.BackConstants.k_ampFeedPower);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_frontSubsystem.stop();
    m_backSubsystem.stop();
    m_pivotSubsystem.setPositionDegrees(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
