
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoOrientCommand extends Command {
  /** Creates a new AutoOrientCommand. */
  DriveSubsystem m_subsystem;
  DoubleSupplier m_y;
  DoubleSupplier m_x;
  double m_angle;
  double k_deadzone = 2.5;

  private double k_f = .15;
  private double k_p = .013;
  private double k_i = 0;
  private double k_d = 0;
  PIDController m_pid = new PIDController(k_p, k_i, k_d);

  public AutoOrientCommand(DriveSubsystem driveSubsystem, double angle, DoubleSupplier y, DoubleSupplier x) {
    m_subsystem = driveSubsystem;
    m_y = y;
    m_x = x;
    m_angle = angle;
    m_pid.enableContinuousInput(0, 360);
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
    double heading = m_subsystem.getHeadingInDegrees();
    double rot = m_pid.calculate(heading, m_angle);
    rot += (k_f * Math.signum(rot));
    m_subsystem.drive(-y, x, rot, true, true);
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
