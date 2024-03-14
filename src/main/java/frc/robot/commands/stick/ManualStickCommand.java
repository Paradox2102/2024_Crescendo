// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stick;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StickSubsystem;

public class ManualStickCommand extends Command {
  private StickSubsystem m_stickSubsystem;
  private DoubleSupplier m_y;

  /** Creates a new ManualStickCommand. */
  public ManualStickCommand(StickSubsystem subsystem, DoubleSupplier y) {
    m_stickSubsystem = subsystem;
    m_y = y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_stickSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = m_y.getAsDouble();

    if (y < 0) {
      m_stickSubsystem.setPower(1);
    } else if (y > 0) {
      m_stickSubsystem.setPower(-1);
    } else {
      m_stickSubsystem.stop();;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stickSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
