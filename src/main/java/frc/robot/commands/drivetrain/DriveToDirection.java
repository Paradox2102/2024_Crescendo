// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDirection extends Command {
     /** Creates a new ArcadeDrive. */

  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_speed;
  private DoubleSupplier m_direction;

  public DriveToDirection(DriveSubsystem driveSubsystem, DoubleSupplier speed, DoubleSupplier direction) {
    m_subsystem = driveSubsystem;
    m_speed = speed;
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDrive", 0, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_speed.getAsDouble();
    double direction = m_direction.getAsDouble();
    double x = speed*Math.cos(Math.PI/180*direction);
    double y = speed*Math.sin(Math.PI/180*direction);
    double rot = m_subsystem.orientPID(direction);
    m_subsystem.drive(
      x, 
      y, 
      rot, 
      true, 
      true,
      Constants.DriveConstants.k_rotatePoint
    );

    
    // m_swerve.setModuleStates(m_defaultState);
    // System.out.println(String.format("x=%f, y=%f, rot=%f, isFieldRelative=%b", x, y, rot, isFieldRelative)); 
  }
  
  

  // Called once the command ends or is interrupted.
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
