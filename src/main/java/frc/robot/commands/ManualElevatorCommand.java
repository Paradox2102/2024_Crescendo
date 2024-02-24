// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.apriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
  private ElevatorSubsystem m_subsystem;
  private DoubleSupplier m_getY;

  /** Creates a new ManualElevator. */
  public ManualElevatorCommand(ElevatorSubsystem subsystem, DoubleSupplier getY) {
    m_subsystem = subsystem;
    m_getY = getY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ManualElevatorCommand", 0, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //round y to 0 if its within the 0 range
    double y = MathUtil.applyDeadband(m_getY.getAsDouble(), Constants.ElevatorConstants.k_driveDeadband);

    //gets cooked position
    double position = m_subsystem.getCookedElevatorPosition();

    // setting the limit
    if (position >= Constants.ElevatorConstants.k_maxDistance && m_getY.getAsDouble() < 0) {
      m_subsystem.setPower(0);
    } else if (position <= Constants.ElevatorConstants.k_minDistance && m_getY.getAsDouble() > 0) {
      m_subsystem.setPower(0);
    } else if (m_getY.getAsDouble() == 0) {
      m_subsystem.setPower(0);;
    } else {
      m_subsystem.setPower(m_getY.getAsDouble() > 0 ? 1 : -1);
    }
    // if (m_getY.getAsDouble() > 0) {
    //   m_subsystem.setPower(.9);
    // } else if (m_getY.getAsDouble() < 0){
    //   m_subsystem.setPower(-0.9);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
