
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoOrientCommand extends Command {
  /** Creates a new AutoOrientCommand. */
  DriveSubsystem m_subsystem;
  DoubleSupplier m_y;
  DoubleSupplier m_x;
  double m_angle;
  double k_deadzone = 2.5;

  PIDController m_pid = new PIDController(Constants.DriveConstants.k_rotateP, Constants.DriveConstants.k_rotateI, Constants.DriveConstants.k_rotateD);

  public AutoOrientCommand(DriveSubsystem driveSubsystem, double angle, DoubleSupplier y, DoubleSupplier x) {
    m_subsystem = driveSubsystem;
    m_y = y;
    m_x = x;
    m_angle = angle;
    m_pid.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double rot = m_subsystem.orientPID(m_angle);
    m_subsystem.drive(y, -x, rot, true, true, Constants.DriveConstants.k_rotatePoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getHeadingInDegrees() - m_angle) <= k_deadzone;
  }
}
