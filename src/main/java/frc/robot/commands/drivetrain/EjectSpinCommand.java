// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class EjectSpinCommand extends Command {
  DriveSubsystem m_driveSubsystem;
  PivotSubsystem m_pivotSubsystem;
  private DoubleSupplier m_speed;
  /** Creates a new EjectSpinCommand. */
  public EjectSpinCommand(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem, DoubleSupplier speed) {
    m_driveSubsystem = driveSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_speed = speed;
    addRequirements(m_driveSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPositionDegrees(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {// .336
    m_driveSubsystem.drive(0, 0, m_speed.getAsDouble(), true, true, new Translation2d(.1, 0.236));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
