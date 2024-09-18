// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PassingAim extends Command {
  private PivotSubsystem m_pivotSubsystem;
  private ManipulatorSubsystem m_shooterSubsystem;
  private DriveSubsystem m_driveSubsystem;
  /** Creates a new PassingAim. */
  public PassingAim(PivotSubsystem pivotSubsystem, ManipulatorSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    m_pivotSubsystem = pivotSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivotSubsystem, m_shooterSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPositionDegrees(m_pivotSubsystem.getPivotAngleFromDistance(m_driveSubsystem.getTranslationalDistanceFromCornerMeters()));
    m_shooterSubsystem.setVelocityRPM(m_shooterSubsystem.getPassSpeed());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterSubsystem.isReady();
  }
}
