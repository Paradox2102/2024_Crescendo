// Copyright (c) FIRST, Parthakkathar Softwares, and other WPIPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class JukeShotRotateAim extends Command {
  /** Creates a new JukeShotRotateAim. */
  DriveSubsystem m_subsystem;
  boolean m_left;
  Translation2d m_rotatePoint = new Translation2d();
  Timer m_timer = new Timer();
  public JukeShotRotateAim(DriveSubsystem driveSubsystem, boolean left) {
    m_subsystem = driveSubsystem;
    m_left = left;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotatePoint = m_left ? new Translation2d(-.326, -.326) : new Translation2d(-.326, .326); 
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = MathUtil.applyDeadband(m_subsystem.orientPID(m_subsystem.getFutureRotationalGoalFromTargetDegrees()), 0);
    // Force it to rotate the way we want depending on which corner we are rotating from
    rot *= (Math.signum(rot) == (m_left ? -1 : 1) ? 1 : -1);
    m_subsystem.drive(
      0, 
      0, 
      rot, 
      true, 
      true,
      m_rotatePoint
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(
      0, 
      0, 
      0, 
      true, 
      true,
      Constants.DriveConstants.k_rotatePoint
    );
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getRotationDistanceFromTargetError()) < 25;
    // return m_timer.get() > 1;
  }
}
