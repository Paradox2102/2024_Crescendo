package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.apriltagsCamera.ApriltagLocation;
import frc.apriltagsCamera.ApriltagsCamera;
import frc.apriltagsCamera.Logger;
import frc.apriltagsCamera.PositionServer;
import frc.robot.subsystems.DriveSubsystem;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTrackerPose {
	private SwerveDrivePoseEstimator m_poseEstimator;
	private DriveSubsystem m_driveSubsystem;
	public PositionServer m_posServer;

	public PositionTrackerPose(PositionServer posServer, double x, double y, DriveSubsystem driveSubsystem) {
		super();
		m_driveSubsystem = driveSubsystem;
		m_posServer = posServer;

		// For the extended constructor, the default values are:
		// VecBuilder.fill(0.02, 0.02, 0.01) - SD of internal state
		// VecBuilder.fill(0.1, 0.1, 0.1)) - SD of vision pose measurment
		// Based on the experiments performed on 2023-03-17 Friday, the vison pose
		// estimates (in metres, metres, radians) should be between:
		// env vs cam: 0.038299922 0.03770631627 0.01169028658
		// est vs cam: 0.01801886372 0.01776299463 0.007439846418
		// As these are significantly lower than the defaults, using them would make us
		// trust the camera more.
		// Slicing the data by distance, I see a stong linear relationship between
		// distance and SD for all of X, Y, and angle,
		// but at the maximum distance the errors are still less than the default.
		// -Gavin

		m_poseEstimator = new SwerveDrivePoseEstimator(
				m_driveSubsystem.getSwerve(),
				m_driveSubsystem.getGyroRotation2d(),
				m_driveSubsystem.getModulePosition(),
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)), ApriltagsCamera.k_odometrySD,
				ApriltagsCamera.k_visionSD);

		// m_posServer = new PositionServer();
		// m_posServer.start();
	}

	public Pose2d getPose2d() {
		return m_poseEstimator.getEstimatedPosition();
	}

	public void setXYAngle(double x, double y, double angleInDegrees) {
		Logger.log("PositionTracker", 1, String.format("x=%f, y=%f, angle=%f", x, y, angleInDegrees));
		m_poseEstimator.resetPosition(
				Rotation2d.fromDegrees(angleInDegrees),
				m_driveSubsystem.getModulePosition(),
				new Pose2d(x, y, Rotation2d.fromDegrees(angleInDegrees)));
	}

	public static class PositionContainer {
		public double x, y;

		public PositionContainer(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}

	ApriltagLocation tags[] = { new ApriltagLocation(1, 0, 0, -90) };

	public void update(ApriltagsCamera frontCamera, ApriltagsCamera rearCamera) {
		m_poseEstimator.updateWithTime(
				ApriltagsCamera.getTime(),
				m_driveSubsystem.getGyroRotation2d(),
				m_driveSubsystem.getModulePosition());
		if (!DriverStation.isAutonomous()) {
			// frontCamera.processRegions(m_poseEstimator);
			// rearCamera.processRegions(m_poseEstimator);
		}

		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			// m_posServer.setAllianceColor(alliance.get() == DriverStation.Alliance.Red);
		}
		Pose2d pos = m_poseEstimator.getEstimatedPosition();
		SmartDashboard.putNumber("xPos", pos.getX());
		SmartDashboard.putNumber("yPos", pos.getY());
		SmartDashboard.putNumber("Robot Angle", pos.getRotation().getDegrees());
		m_posServer.setPosition(pos.getX(), pos.getY(), pos.getRotation().getDegrees());

		// PositionServer.Target target = m_posServer.getTarget();

		// if (target != null) {
		// 	// Logger.log("PositionTracker", 1, String.format("x=%f,y=%f,h=%f", target.m_x,
		// 	// target.m_y, target.m_h));
		// 	SmartDashboard.putNumber("TargetX", target.m_x);
		// 	SmartDashboard.putNumber("TargetY", target.m_y);
		// 	SmartDashboard.putNumber("TargetH", target.m_h);
		// }
	}

	// private void logUpdate()
	// {
	// 	Rotation2d rotation = m_driveSubsystem.getGyroRotation2d();
	// 	SwerveModulePosition[] modules = m_driveSubsystem.getModulePosition();

	// 	Logger.log("PositionTrackerPose", 1, String.format("rot=%f,m0=%f,m1=%f,m2=%f,m3=%f,a0=%f,a1=%f,a2=%f,a3=%f", 
	// 				rotation.getDegrees(),
	// 				modules[0].distanceMeters,
	// 				modules[1].distanceMeters,
	// 				modules[2].distanceMeters,
	// 				modules[3].distanceMeters,
	// 				modules[0].angle.getDegrees(),
	// 				modules[1].angle.getDegrees(),
	// 				modules[2].angle.getDegrees(),
	// 				modules[3].angle.getDegrees()));

	// }

	public void update(ApriltagsCamera camera) {
		// logUpdate();
		m_poseEstimator.updateWithTime(
				ApriltagsCamera.getTime(),
				m_driveSubsystem.getGyroRotation2d(),
				m_driveSubsystem.getModulePosition());
		// Pose2d pose = m_poseEstimator.getEstimatedPosition();
		// Logger.log("PositionTrackerPose", 1, String.format("x=%f,y=%f,a=%f", 
		// 								pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
		// if (!DriverStation.isAutonomous()) {
		camera.processRegions(m_poseEstimator);
		// }
		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			// m_posServer.setAllianceColor(alliance.get() == DriverStation.Alliance.Red);
		}

		Pose2d pos = m_poseEstimator.getEstimatedPosition();
		SmartDashboard.putNumber("xPos", pos.getX());
		SmartDashboard.putNumber("yPos", pos.getY());
		SmartDashboard.putNumber("Robot Angle", pos.getRotation().getDegrees());
		m_posServer.setPosition(pos.getX(), pos.getY(), pos.getRotation().getDegrees());

		// PositionServer.Target target = m_posServer.getTarget();

		// if (target != null) {
		// 	// Logger.log("PositionTracker", 1, String.format("x=%f,y=%f,h=%f", target.m_x,
		// 	// target.m_y, target.m_h));
		// 	SmartDashboard.putNumber("TargetX", target.m_x);
		// 	SmartDashboard.putNumber("TargetY", target.m_y);
		// 	SmartDashboard.putNumber("TargetH", target.m_h);
		// }
	}
}