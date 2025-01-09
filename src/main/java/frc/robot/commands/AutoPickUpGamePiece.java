// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPickUpGamePiece extends Command {
  DriveSubsystem m_driveSubsystem;
  ManipulatorSubsystem m_holderSubsystem;
  ManipulatorSubsystem m_shooterSubsystem;
  PivotSubsystem m_pivotSubsystem;

  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_rot;

  private final double k_f = .1;
  private final double k_p = 0;
  private final double k_i = 0;
  private final double k_d = 0;
  private final double k_iZone = 10;
  PIDController m_pid = new PIDController(k_p, k_i, k_d);

  double k_deadzone = 30;
  double k_minPower = .1;
  double m_setpoint = 0;

  /** Creates a new AutoPickUpGamePiece. */
  public AutoPickUpGamePiece(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem, ManipulatorSubsystem shooterSubsystem, ManipulatorSubsystem holderSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    m_driveSubsystem = driveSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_holderSubsystem = holderSubsystem;
    m_x = x;
    m_y = y;
    m_rot = rot;
    m_pid.setIZone(k_iZone);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_holderSubsystem, m_shooterSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_intakePositionDegrees);
    m_shooterSubsystem.setVelocityRPM(Constants.ShooterConstants.k_intakeVelocityRPM);
    m_holderSubsystem.setVelocityRPM(Constants.HolderConstants.k_intakeVelocityRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceFromTarget = m_driveSubsystem.getRotationalDistanceFromGamePiece();
    double rot;
    boolean seeGamePiece;
    if (distanceFromTarget == 99999999) {
      rot = m_rot.getAsDouble();
      seeGamePiece = false;
    } else {
      seeGamePiece = true;
      double target = m_driveSubsystem.getTargetCenter();
      rot = (k_f * -Math.signum(320-target)) - m_pid.calculate(target, 320);
      rot = Math.abs(target-320) <= k_deadzone ? 0 : rot;
    }
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    if (!seeGamePiece) {
      m_driveSubsystem.drive(x, y, -rot, true, true, Constants.DriveConstants.k_rotatePoint);
    } else {
      x = Math.abs(x);
      y = Math.abs(y);
      double input = x > y ? x : y;
      m_driveSubsystem.drive(input, 0, -rot, false, true, Constants.DriveConstants.k_rotatePoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.setPositionDegrees(Constants.PivotConstants.k_resetPositionDegrees);
    m_shooterSubsystem.stop();
    m_holderSubsystem.stop();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
