// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetSpeakerAmpMode;
import frc.robot.commands.gamePieceManipulation.ShootCommand;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPreload extends SequentialCommandGroup {
  /** Creates a new ShootPreload. */
  public ShootPreload(PivotSubsystem pivotSubsystem, ManipulatorSubsystem frontRollers, ManipulatorSubsystem backRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartFront(),
      new SetSpeakerAmpMode(true),
      new SetPivotPos(pivotSubsystem, Constants.PivotConstants.k_resetPositionDegrees),
      new WaitCommand(3),
      new ShootCommand(frontRollers, backRollers)
    );
  }
}
