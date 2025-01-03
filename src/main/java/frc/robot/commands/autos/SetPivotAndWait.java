// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.SetPivotOffRobotLocation;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotAndWait extends SequentialCommandGroup {
  /** Creates a new SetPivotAndWait. */
  PivotSubsystem m_pivotSubsystem;
  public SetPivotAndWait(PivotSubsystem pivotSubsystem) {
    m_pivotSubsystem = pivotSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetPivotOffRobotLocation(m_pivotSubsystem), new WaitCommand(.25));
  }
}
