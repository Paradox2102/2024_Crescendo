// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.apriltagsCamera.PositionServer;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.ApriltagsCamera.ApriltagsCameraType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StopEverything;
import frc.robot.commands.ToggleAutoAim;
import frc.robot.commands.PassShot;
import frc.robot.commands.ToggleShootSideCommand;
import frc.robot.commands.apriltags.SetApriltagsDashboard;
import frc.robot.commands.apriltags.SetApriltagsLogging;
import frc.robot.commands.autos.BackFeedCommand;
import frc.robot.commands.autos.CountBulldoze;
import frc.robot.commands.autos.IntakeAndGoToBackShooter;
import frc.robot.commands.autos.RevBackShooter;
import frc.robot.commands.autos.StartBack;
import frc.robot.commands.autos.StartFront;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.AutoOrientCommand;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.EjectSpinCommand;
import frc.robot.commands.elevator.ManualElevatorCommand;
import frc.robot.commands.gamePieceManipulation.DefaultManipulatorCommand;
import frc.robot.commands.gamePieceManipulation.EjectGamePiece;
import frc.robot.commands.gamePieceManipulation.FeedCommand;
import frc.robot.commands.gamePieceManipulation.IntakeCommand;
import frc.robot.commands.gamePieceManipulation.JukeShot;
import frc.robot.commands.gamePieceManipulation.ResetSubsystemsCommand;
import frc.robot.commands.gamePieceManipulation.RevCommand;
import frc.robot.commands.gamePieceManipulation.ShootCommand;
import frc.robot.commands.gamePieceManipulation.ShootSequence;
import frc.robot.commands.pivot.DefaultPivotCommand;
import frc.robot.commands.pivot.SetPivotOffInputDistance;
import frc.robot.commands.stick.SetStickPos;
import frc.robot.commands.test.D2Intake;
import frc.robot.led.LEDConfig;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSensors;
import frc.robot.subsystems.StickSubsystem;
import frc.triggers.HoldTrigger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  ApriltagsCamera m_apriltagCamera = new ApriltagsCamera();
  ApriltagsCamera m_apriltagCameraSide = new ApriltagsCamera();
  PositionServer m_posServer = new PositionServer(m_apriltagCamera);
  Constants m_constants = new Constants();
  LEDConfig m_ledConfig;

  final ShooterSensors m_shooterSensors = new ShooterSensors();
  final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_apriltagCamera, m_apriltagCameraSide);
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(() -> m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters());
  private final ManipulatorSubsystem m_frontSubsystem = new ManipulatorSubsystem(true, () -> m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters());
  private final ManipulatorSubsystem m_backSubsystem = new ManipulatorSubsystem(false, () -> m_driveSubsystem.getFutureTranslationDistanceFromSpeakerMeters());
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final StickSubsystem m_stickSubsystem = new StickSubsystem();

  private final CommandJoystick m_joystick = new CommandJoystick(1);
  //private final CommandJoystick m_testStick = new CommandJoystick(2);
  public final PositionTrackerPose m_tracker = new PositionTrackerPose(m_posServer, 0, 0, m_driveSubsystem);
  SendableChooser<Command> m_autoSelection = new SendableChooser<>();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    // Configure the trigger bindings
    configureBindings();
    m_ledConfig = new LEDConfig(robot, m_apriltagCamera, m_apriltagCameraSide);

    m_driveSubsystem.setTracker(m_tracker);
    NamedCommands.registerCommand("intake", new IntakeCommand(m_backSubsystem, m_frontSubsystem, m_pivotSubsystem));
    NamedCommands.registerCommand("intake back", new IntakeAndGoToBackShooter(m_frontSubsystem, m_pivotSubsystem, 1.8));
    NamedCommands.registerCommand("rev shooter", new RevCommand(m_frontSubsystem, m_backSubsystem));
    NamedCommands.registerCommand("rev back", new RevBackShooter(m_backSubsystem));
    NamedCommands.registerCommand("switch to shoot back", new ToggleShootSideCommand(false));
    NamedCommands.registerCommand("switch to shoot front", new ToggleShootSideCommand(true));
    NamedCommands.registerCommand("start back", new StartBack());
    NamedCommands.registerCommand("start front", new StartFront());
    NamedCommands.registerCommand("shoot", new ShootCommand(m_frontSubsystem, m_backSubsystem));
    NamedCommands.registerCommand("feedthrough", new FeedCommand(m_frontSubsystem, m_backSubsystem));
    NamedCommands.registerCommand("back feed", new BackFeedCommand(m_frontSubsystem));
    NamedCommands.registerCommand("stop everything", new StopEverything(m_driveSubsystem, m_frontSubsystem, m_backSubsystem, m_pivotSubsystem));
    NamedCommands.registerCommand("reset everything", new ResetSubsystemsCommand(m_pivotSubsystem, m_frontSubsystem, m_backSubsystem));
    NamedCommands.registerCommand("bulldoze counter", new CountBulldoze(m_frontSubsystem, m_backSubsystem, m_pivotSubsystem));
    // Aim
    NamedCommands.registerCommand("subwoofer aim", new SetPivotOffInputDistance(m_pivotSubsystem, 1.5));
    NamedCommands.registerCommand("four piece aim", new SetPivotOffInputDistance(m_pivotSubsystem, 2));
    NamedCommands.registerCommand("source 3 start aim", new SetPivotOffInputDistance(m_pivotSubsystem, 1.623));
    NamedCommands.registerCommand("source/amp 3 aim", new SetPivotOffInputDistance(m_pivotSubsystem, 3.77));
    NamedCommands.registerCommand("auto aim", new DefaultPivotCommand(m_pivotSubsystem, m_driveSubsystem, true));


    // Auto Chooser
    m_autoSelection.addOption("Nothing", new InstantCommand());
    m_autoSelection.addOption("Amp Side 5", new PathPlannerAuto("amp side 5"));
    m_autoSelection.addOption("Wing 4 Piece PHR", new PathPlannerAuto("wing 4 piece source up"));
    m_autoSelection.addOption("Wing 4 Piece RHP", new PathPlannerAuto("wing 4 piece amp down"));
    m_autoSelection.addOption("Source Side 4", new PathPlannerAuto("source side 4"));
    m_autoSelection.addOption("Amp Side Rush", new PathPlannerAuto("amp 3 rush"));
    SmartDashboard.putData(m_autoSelection);

    // m_apriltagCamera.setCameraInfo(8.375, 12, 180, ApriltagsCameraType.GS_6mm); // y = 6
    // m_apriltagCamera.setCameraInfo(5.125, 15.5, 0, ApriltagsCameraType.GS_6mm); // y = 9.5
    // Front
    m_apriltagCamera.setCameraInfo(Constants.DriveConstants.k_cameraFrontX, Constants.DriveConstants.k_cameraFrontY, 180, ApriltagsCameraType.GS_6mm); // y = 6
    // Back
    m_apriltagCamera.setCameraInfo(Constants.DriveConstants.k_cameraBackX, Constants.DriveConstants.k_cameraBackY, 0, ApriltagsCameraType.GS_6mm); // y = 9.5
    m_apriltagCamera.connect("10.21.2.12", 5800);
    

    m_apriltagCameraSide.setCameraInfo(Constants.DriveConstants.k_cameraRightX, Constants.DriveConstants.k_cameraRightY, -90, ApriltagsCameraType.GS_6mm); 
    m_apriltagCameraSide.setCameraInfo(Constants.DriveConstants.k_cameraLeftX, Constants.DriveConstants.k_cameraLeftY, 88, ApriltagsCameraType.GS_6mm); 
    //m_apriltagCameraSide.connect("10.21.2.12", 5800);

    m_posServer.start();
  }

  private boolean getPositionServerButtonState(int button) {
    return m_posServer.getButtonState(button);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(() -> getPositionServerButtonState(1)).onTrue(new SetApriltagsLogging(m_apriltagCamera, m_apriltagCameraSide, true));
    new Trigger(() -> getPositionServerButtonState(2)).onTrue(new SetApriltagsLogging(m_apriltagCamera, m_apriltagCameraSide, false));
    new Trigger(() -> getPositionServerButtonState(3)).onTrue(new SetApriltagsDashboard(m_apriltagCamera, m_apriltagCameraSide, true));
    new Trigger(() -> getPositionServerButtonState(4)).onTrue(new SetApriltagsDashboard(m_apriltagCamera, m_apriltagCameraSide, false));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    HoldTrigger m_slowMode = new HoldTrigger(m_driverController.rightBumper());
    HoldTrigger m_slowMode1 = new HoldTrigger(m_driverController.leftBumper());
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
      m_driveSubsystem, 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getRightX(),
      new Trigger(m_slowMode),
      new Trigger(m_slowMode1)
    ));

    m_pivotSubsystem.setDefaultCommand(new DefaultPivotCommand(m_pivotSubsystem, m_driveSubsystem, true));
    m_frontSubsystem.setDefaultCommand(new DefaultManipulatorCommand(m_frontSubsystem, m_driveSubsystem, true));
    m_backSubsystem.setDefaultCommand(new DefaultManipulatorCommand(m_backSubsystem, m_driveSubsystem, false));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.leftTrigger().toggleOnTrue(new ShootCommand(m_shooterSubsystem, m_backSubsystem));
    m_driverController.leftTrigger().toggleOnTrue(new ShootSequence(m_frontSubsystem, m_backSubsystem, m_stickSubsystem));
    m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_backSubsystem, m_frontSubsystem, m_pivotSubsystem));
        
    m_driverController.a().onTrue(new PassShot(m_driveSubsystem, m_frontSubsystem, m_backSubsystem, m_pivotSubsystem, m_driverController));

    //ToggleTrigger shootIntake = new ToggleTrigger(m_joystick.button(7));
    m_joystick.button(1).toggleOnTrue(new RevCommand(m_frontSubsystem, m_backSubsystem));
    m_joystick.button(2).whileTrue(new D2Intake(m_frontSubsystem, m_backSubsystem, true));
    m_joystick.button(4).whileTrue(new EjectGamePiece(m_pivotSubsystem, m_frontSubsystem, m_backSubsystem));
    m_joystick.button(7).onTrue(new InstantCommand(() -> {Constants.States.m_shootIntakeSide = !Constants.States.m_shootIntakeSide;}));
    m_joystick.button(8).toggleOnTrue(new FeedCommand(m_frontSubsystem, m_backSubsystem));
    m_joystick.button(9).whileTrue(new ManualElevatorCommand(m_elevatorSubsystem, () -> m_joystick.getY()));
    m_joystick.button(10).whileTrue(new EjectSpinCommand(m_driveSubsystem, m_pivotSubsystem, () -> m_joystick.getY()));
    m_joystick.button(11).onTrue(new StopEverything(m_driveSubsystem, m_frontSubsystem, m_backSubsystem, m_pivotSubsystem));
    m_joystick.button(6).onTrue(new ToggleAutoAim());
    m_joystick.button(10).onTrue(new InstantCommand(() -> {Constants.States.m_shootIntakeSide = !Constants.States.m_shootIntakeSide;}));
    m_joystick.button(3).whileTrue(new SetStickPos(m_stickSubsystem, false));

    m_joystick.button(5).whileTrue(new DriveToPosition(m_driveSubsystem, new Pose2d(2.86, 7.16, Rotation2d.fromDegrees(0))));
    //ToggleTrigger m_brakeMode = new ToggleTrigger(m_joystick.button(12));
    //m_joystick.button(12).onTrue(new SetRobotBreakMode(new Trigger(m_brakeMode), m_driveSubsystem, m_pivotSubsystem, m_shooterSubsystem, m_backSubsystem, m_elevatorSubsystem, m_stickSubsystem));
  }
  public double getThrottle() {
    return m_joystick.getThrottle();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoSelection.getSelected();
  }
}
