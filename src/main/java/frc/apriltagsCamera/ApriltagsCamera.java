/*
 *	  Copyright (C) 2022  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */

package frc.apriltagsCamera;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.ParadoxField;

/**
 * 
 * @author John Gaby
 * 
 * @brief The ApriltagsCamera class handles communication with the Raspberry Pi
 *        Camera.
 * 
 *        This class allows you to connect to the Data server of the Raspberry
 *        Pi and receive information about the image frames processed by the
 *        camera.
 *
 *        It also implements a time sync protocol to synchronize the clocks and
 *        performs keep-alive functions that detect a disconnect.
 *
 */
public class ApriltagsCamera implements frc.apriltagsCamera.Network.NetworkReceiver {
	// public Pose2d m_pose2d = new Pose2d(0, 0, new Rotation2d(0));
	private boolean m_log = false;
	private static double k_maxLogTime = 2.5 * 60; // Length of a match
	private edu.wpi.first.wpilibj.Timer m_logTimer = new edu.wpi.first.wpilibj.Timer();

	public static final Vector<N3> k_odometrySD = VecBuilder.fill(0.1, 0.1, 0.1); // Default odometry standard
																					// deviations
	// public static final Vector<N3> k_visionSD = VecBuilder.fill(0.1, 0.1, 0.1); // Default vision standerd devations
//	public static final Vector<N3> k_visionSD = VecBuilder.fill(0.05, 0.05, 0.05); // Default vision standerd devations
	public static final Vector<N3> k_visionSD = VecBuilder.fill(0.5, 0.5, 1.0); // Default vision standerd devations

	private static final double k_minSDAdjustDistance = 0.5; // Minimum distance for which we apply the standard
																// deviation adjustment
	private static final double k_maxSDAdjustDistance = 4.0; // Maximum distance for which we apply the stander
																// deviation adjustment
	private static final double k_maxAngleDistance = 5.0; // If this distance is greater than this value, do not update
															// the estimated angle
	private static final double k_SDAdjustSlope = 8.0; // Slop of adjustment between min and max distances
	private static final double k_maxDistance = 12.0; // Max distance beyond which the camera is completely unreliable
	private static final double k_maxAngleError = 2; // Max acceptable angle error in degrees

	/**
	 * @brief The ApriltagsCameraStats class collects the current camera performance
	 *        statistics.
	 *
	 */
	public class ApriltagsCameraStats {
		public int m_averageDelay = 0; // !<Specifies the average frame delay
		public int m_maxDelay = 0; // !<Specifies the max frame delay
		public int m_minDelay = Integer.MAX_VALUE; // !<Specifies the min frame delay
		public int m_lostFrames; // !<Specifies the number of lost frames
		public long m_time; // !<Specifies the elapsed time

		public ApriltagsCameraStats(int avgDelay, int maxDelay, int minDelay, int lostFrames, long time) {
			m_averageDelay = avgDelay;
			m_maxDelay = maxDelay;
			m_minDelay = minDelay;
			m_lostFrames = lostFrames;
			m_time = time;
		}
	}

	class ApriltagsCameraInfo {
		ApriltagsCameraRegions m_regions;
		final double m_xOffsetInches;
		final double m_yOffsetInches;
		final double m_cameraAngleDegrees;
		int m_frameCount = 0;
		int m_missingCount = 0;
		int m_lastFrame = -1;

		ApriltagsCameraInfo(double xOffsetInches, double yOffsetInches, double cameraAngleDegrees) {
			m_xOffsetInches = xOffsetInches;
			m_yOffsetInches = yOffsetInches;
			m_cameraAngleDegrees = cameraAngleDegrees;
		}
	}

	private class ApriltagPosition {
		double m_time;
		Pose2d m_estPos;
		// Pose2d m_camPos;
	}

	private class ApriltagsQueue {
		final static int k_queueSize = 32;
		int m_position = 0;
		ApriltagPosition[] m_queue = new ApriltagPosition[k_queueSize];

		ApriltagsQueue() {
			for (int i = 0; i < k_queueSize; i++) {
				m_queue[i] = new ApriltagPosition();
			}
		}

		void add(Pose2d estPos, /* Pose2d camPos, */ double time) {
			m_queue[m_position].m_time = time;
			m_queue[m_position].m_estPos = estPos;
			// m_queue[m_position].m_camPos = camPos;

			m_position = (m_position + 1) % k_queueSize;
		}

		ApriltagPosition findPosition(double time) {
			int position = (m_position + k_queueSize - 1) % k_queueSize;
			while (position != m_position && m_queue[position].m_estPos != null) {
				if (time > m_queue[position].m_time) {
					// Logger.log("ApriltagsQueue", 1, String.format("found at %d", (m_position -
					// position + k_queueSize) % k_queueSize));
					return (m_queue[position]);
				}
				position = (position + k_queueSize - 1) % k_queueSize;
			}

			return (null);
		}
	}

	// static final double k_maxAngleDeviation = 1.5; // Max legitimate angle change
	// between samples
	static final int k_maxCameras = 2;
	static final int k_maxTags = 16;
	ApriltagsQueue m_queue = new ApriltagsQueue(); // [][] = new ApriltagsQueue[k_maxCameras][k_maxTags];
	boolean m_dashboard = true;

	/**
	 * @brief The ApriltagsCameraRegion specifies a single detected region
	 *
	 */
	public class ApriltagsCameraRegion {
		private ApriltagsCameraInfo m_info;
		public int m_tag; // !<Specifies the region color [0..3]
		// public double m_rvec[] = new double[3];
		public double m_yaw;
		public double m_pitch;
		public double m_roll;
		public double m_tvec[] = new double[3];
		public double m_corners[][] = new double[4][2];
		public double m_relAngleInDegrees;
		public double m_angleInDegrees;
		public double m_angleOffset;

		// ! @cond PRIVATE
		public ApriltagsCameraRegion(ApriltagsCameraInfo info, int tag, double yaw, double pitch, double roll,
				double t0, double t1, double t2,
				double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
			m_info = info;
			m_tag = tag;
			// m_rvec[0] = r0;
			// m_rvec[1] = r1;
			// m_rvec[2] = r2;
			m_yaw = yaw;
			m_pitch = pitch;
			m_roll = roll;
			m_tvec[0] = t0;
			m_tvec[1] = t1;
			m_tvec[2] = t2;
			m_corners[0][0] = x0;
			m_corners[0][1] = y0;
			m_corners[1][0] = x1;
			m_corners[1][1] = y1;
			m_corners[2][0] = x2;
			m_corners[2][1] = y2;
			m_corners[3][0] = x3;
			m_corners[3][1] = y3;

			m_angleInDegrees = m_relAngleInDegrees = yaw; // * 52; // + m_cameraAngleDegrees;
			// m_angleOffset = m_angleOffsetInDegrees;
		}
		// ! @endcond

		// Translates the position from the camera position to the center of the robot.
		Pose2d translatePos2d(double xPos, double yPos, double angleInRadians) {
			double a = Math.atan2(m_info.m_yOffsetInches, m_info.m_xOffsetInches);
			double d = Math.sqrt(
					m_info.m_xOffsetInches * m_info.m_xOffsetInches + m_info.m_yOffsetInches * m_info.m_yOffsetInches);
			double b = Math.PI / 2 - angleInRadians - a;
			double dx = d * Math.cos(b);
			double dy = d * Math.sin(b);

			// Logger.log("ApriltagsCamera", 1, String.format("translatePos: dx=%f,dy=%f",
			// dx, dy));

			return new Pose2d(xPos - dx / ApriltagLocations.k_inPerM, yPos + dy / ApriltagLocations.k_inPerM,
					Rotation2d.fromRadians(angleInRadians));
		}

		/*
		 * Computes position of the robot and updates the PoseEstimator
		 *
		 */
		private void updatePosition(
				int cameraNo,
				SwerveDrivePoseEstimator poseEstimator,
				long captureTime,
				int frameNo) {

			// if (cameraNo == 1) return;

			double time = convertTime(captureTime);
			ApriltagLocation tag = ApriltagLocations.findTag(m_tag);
			// ApriltagsQueue queue = null;
			ApriltagPosition lastPos = null;
			Pose2d estPos = poseEstimator.getEstimatedPosition();
			double cameraAngle; // Actual angle returned from the camera;
			double calculateAngle; // Angle used to calculate the x & y positions
			double updateAngle; // Angle used to update the position estimator

			// if ((cameraNo >= 0) && (cameraNo < k_maxCameras) && (m_tag >= 0) && (m_tag <
			// k_maxTags)) {
			// queue = m_queue[cameraNo][m_tag];
			// lastPos = queue.findPosition(time);
			// } else {
			// Logger.log("ApriltagsCamera", 3, String.format("Invalid camera %d or tag %d",
			// cameraNo, m_tag));
			// }
			lastPos = m_queue.findPosition(time);

			if (tag != null) {
				double adjust = 1.0;
				double cx = m_tvec[0] / ApriltagLocations.k_inPerM; // Convert inches to meters
				double cz = m_tvec[2] / ApriltagLocations.k_inPerM; // Convert inches to meters

				updateAngle = calculateAngle = cameraAngle = m_angleInDegrees = normalizeAngle(
						m_relAngleInDegrees + tag.m_targetAngleDegrees
								- m_info.m_cameraAngleDegrees);

				double d = Math.sqrt(cx * cx + cz * cz);

				if (d > k_maxDistance) {
					return;
				}

				// Compute standard deviation parameters to be used based on the distance
				Vector<N3> visionSD = VecBuilder.fill(k_visionSD.get(0, 0), k_visionSD.get(1, 0), k_visionSD.get(2, 0));

				if (d >= k_minSDAdjustDistance) {
					// if (d > k_maxSDAdjustDistance) {
					// 	adjust = 1.0 + k_SDAdjustSlope * (k_maxSDAdjustDistance - k_minSDAdjustDistance);
					// } else {
						adjust = 1.0 + k_SDAdjustSlope * (d - k_minSDAdjustDistance);
					// }
				}

				double lastAngle;
				if (lastPos != null) {
					lastAngle = lastPos.m_estPos.getRotation().getDegrees();
				} else {
					lastAngle = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
				}

				if ((d > k_maxAngleDistance) || (Math.abs(normalizeAngle(lastAngle - cameraAngle)) > k_maxAngleError)) {
					// If max error is exceeded, increase the standard deviation adjustment and use
					// the estimated angle for calculations
					adjust *= 2;
					calculateAngle = lastAngle;

					if (d > k_maxAngleDistance) {
						// If the robot is too far away, do not use the last angle to update the
						// estimator
						updateAngle = lastAngle;
					}
				}

				if (adjust != 1.0) {
					// Only need to adjust the angle parameter
					visionSD.set(2, 0, visionSD.get(2, 0) * adjust);
				}

				if (d > k_maxSDAdjustDistance) {
					// For distances greater that the max distance, use the estimated angle for
					// calculations
					calculateAngle = lastAngle;
				}

				double a = Math.atan2(cx, cz);
				double b = Math.toRadians(calculateAngle) - Math.PI / 2 - a;

				double dy = d * Math.sin(b);
				double dx = d * Math.cos(b);

				// Adjust for camera orientation
				double sin = Math.sin(Math.toRadians(m_info.m_cameraAngleDegrees));
				double cos = Math.cos(Math.toRadians(m_info.m_cameraAngleDegrees));

				double dxp = dy * cos + dx * sin;
				double dyp = dx * cos - dy * sin;

				double xPos = (tag.m_xMeters + dxp);
				double yPos = (tag.m_yMeters - dyp);

				Pose2d calculatedPos = translatePos2d(xPos, yPos, Math.toRadians(normalizeAngle(calculateAngle)));

				// double ex = pos.getX() - currentPose.getX();
				// double ey = pos.getY() - currentPose.getY();
				boolean invalid;

				/*
				 * MUSTFIX
				 * if ((Math.abs(ex) > 1) || (Math.abs(ey) > 1)) {
				 * invalid = m_invalidCount < 3;
				 * Logger.log("ApriltagsCamera", 1,
				 * String.format("Spurious tag: id=%d fn=%d: %f,%f cnt=%d invalid=%b",
				 * m_tag, frameNo, ex, ey, m_invalidCount, invalid));
				 * m_invalidCount++;
				 * } else {
				 * invalid = false;
				 * m_invalidCount = 0;
				 * }
				 */

				// boolean actual = (m_angleInDegrees == cameraAngle);
				invalid = false; // MUSTFIX
				if (!invalid) {
					if (m_log) {
						if (m_logTimer.get() >= k_maxLogTime) {
							m_log = false;
							Logger.closeLogFile("ApriltagsCameraLog");
							Logger.log("ApriltagsCamera", 3, "Max log time reached");
						} else {
							Logger.log("ApriltagsCameraLog", 1,
									String.format(",%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f", m_tag,
											lastAngle, cameraAngle, calculateAngle, updateAngle,
											estPos.getRotation().getDegrees(),
											calculatedPos.getX(), estPos.getX(),
											calculatedPos.getY(), estPos.getY(),
											adjust,	frameNo, d));
						}
					}

					if (!m_camerasDisabled) {
						// Add vision measurment using the updateAngle
						poseEstimator.addVisionMeasurement(
								new Pose2d(calculatedPos.getX(), calculatedPos.getY(),
										Rotation2d.fromDegrees(updateAngle)),
								time, visionSD);
					}

					// if (queue != null) {
					// Put the uncorrected camera angle into the queue
					m_queue.add(estPos, time);
					// }

					if (m_dashboard) {
						// SmartDashboard.putString(String.format("c%d-%d est", cameraNo, m_tag),
						// actual ? "actual" : "estimated");
						SmartDashboard.putNumber(String.format("c%d-%d yaw", cameraNo, m_tag), m_yaw);
						SmartDashboard.putNumber(String.format("c%d-%d dxp", cameraNo, m_tag), dxp);
						SmartDashboard.putNumber(String.format("c%d-%d dyp", cameraNo, m_tag), dyp);
						SmartDashboard.putNumber(String.format("c%d-%d cx", cameraNo, m_tag), calculatedPos.getX());
						SmartDashboard.putNumber(String.format("c%d-%d cy", cameraNo, m_tag), calculatedPos.getY());
						SmartDashboard.putNumber(String.format("c%d-%d ca", cameraNo, m_tag), cameraAngle);
						SmartDashboard.putNumber(String.format("c%d-%d cac", cameraNo, m_tag), calculateAngle);
						SmartDashboard.putNumber(String.format("c%d-%d ex", cameraNo, m_tag), estPos.getX());
						SmartDashboard.putNumber(String.format("c%d-%d ey", cameraNo, m_tag), estPos.getY());
						SmartDashboard.putNumber(String.format("c%d-%d ea", cameraNo, m_tag),
								estPos.getRotation().getDegrees());
						SmartDashboard.putNumber(String.format("c%d-%d ad", cameraNo, m_tag), adjust);
						// SmartDashboard.putNumber(String.format("c%d-%d sd", cameraNo, m_tag),
						// visionSD.get(2, 0));
					}
				}
			} else {
				Logger.log("ApriltagsCamera", 1, String.format("updatePosition: Tag %d not found", m_tag));
			}
		}
	}

	int m_count;

	/**
	 * @brief The ApriltagsCameraRegions specifies list of detected regions
	 *
	 *        The list will contain up to the max regions specified using the
	 *        ImageViewer and will be sorted from the largest to smallest area.
	 *
	 */
	public class ApriltagsCameraRegions {
		// public int m_cameraNo;
		public ApriltagsCameraInfo m_info;
		public int m_targetVertPos; // !<Specifies the vertical target position as set by the ImageViewer
		public int m_targetHorzPos; // !<Specifies the horizontal target position as set by the ImageViewer
		public int m_frameNo; // !<Specifies the frame #
		public int m_width; // !<Specifies the width of the camera image (e.g. 640)
		public int m_height; // !<Specifies the height of the camera image (e.g. 480)
		public int m_lostFrames; // !<Specifies the number of lost frames
		public int m_profile; // Not used
		public long m_captureTime; // !<Specifies the time at which the image was acquired in ms
		public int m_procTime; // !<Specifies the time required to process this image in ms
		public int m_fps; // !<specifies the current frame rate in frames/sec
		public ArrayList<ApriltagsCameraRegion> m_regions = new ArrayList<ApriltagsCameraRegion>();

		// ! @cond PRIVATE
		protected ApriltagsCameraRegions(ApriltagsCameraInfo info, int frameNo, int targetVertPos, int targetHorzPos,
				int width,
				int height,
				int lostFrames,
				long captureTime, int procTime, int fps) {
			Logger.log("ApriltagsCameraRegions", -1,
					String.format("ApriltagsCameraRegions(): width = %d, height = %d", width, height));

			m_info = info;
			m_frameNo = frameNo;
			m_targetVertPos = targetVertPos;
			m_targetHorzPos = targetHorzPos;
			m_width = width;
			m_height = height;
			m_lostFrames = lostFrames;
			m_fps = fps;
			m_captureTime = captureTime;
			m_procTime = procTime;
		}

		protected void addRegion(int tag, double r0, double r1, double r2, double t0, double t1, double t2, double x0,
				double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
			m_regions.add(
					new ApriltagsCameraRegion(m_info, tag, r0, r1, r2, t0, t1, t2, x0, y0, x1, y1, x2, y2, x3, y3));
		}
		// ! @endcond

		/**
		 * Returns a count of the regions
		 * 
		 */
		public int getRegionCount() {
			return (m_regions.size());
		}

		/**
		 * Returns a specified region
		 * 
		 * @param region - Specifies the region to return
		 */
		public ApriltagsCameraRegion getRegion(int region) {

			if ((region >= 0) && (region < m_regions.size())) {

				return (m_regions.get(region));
			}
			;
			return (null);
		}

		/**
		 * Class for the return of the robot position
		 */
		public class RobotPos {
			public double m_x; // Absolute x position of the robot
			public double m_y; // Absolute y position of the robot

			public RobotPos(double x, double y) {
				m_x = x;
				m_y = y;
			}
		}
	}

	private static final int k_syncRetry = 5000;
	private static final int k_syncFirst = 1000;
	// private static final int k_maxCameras = 2;

	private Network m_network = null;
	// private int m_nCameras = 0;
	// private ApriltagsCameraRegions m_regions[] = new
	// ApriltagsCameraRegions[k_maxCameras];
	private ArrayList<ApriltagsCameraInfo> m_cameras = new ArrayList<ApriltagsCameraInfo>();
	private ApriltagsCameraRegions m_nextRegions = null;
	private long m_syncTime;
	private int m_averageDelayCount = 0;
	private int m_averageDelayMax = 30;
	private int m_averageDelaySum = 0;
	private int m_averageDelay = 0;
	private int m_maxDelay = 0;
	private int m_minDelay = Integer.MAX_VALUE;
	private int m_lostFrames;
	private int m_lastLostFrame = 0;
	private long m_startTime = 0;
	private boolean m_connected = false;
	private Timer m_watchdogTimer = new Timer();
	private long m_lastMessage;
	private boolean m_camerasDisabled = false;
	private static final int k_timeout = 5000;
	// private double m_angleOffsetInDegrees = 0;
	boolean m_angleOffsetInitialized = false;
	// private final double m_xOffsetInches;
	// private final double m_yOffsetInches;
	// private final double m_cameraAngleDegrees;
	// private int m_invalidCount;

	private static long k_timeOffset = 0;

	public static double convertTime(long time) {
		if (k_timeOffset == 0) {
			k_timeOffset = System.currentTimeMillis();
		}

		return (time - k_timeOffset) / 1000.0;
	}

	public static double getTime() {
		return (convertTime(System.currentTimeMillis()));
	}

	public ApriltagsCamera() {
		// for (int c = 0; c < k_maxCameras; c++) {
		// for (int t = 0; t < k_maxTags; t++) {
		// m_queue[c][t] = new ApriltagsQueue();
		// }
		// }

		m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				if (m_connected) {
					Logger.log("ApriltagsCamera", -1, "WatchDog");

					m_network.sendMessage("k");

					if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
						Logger.log("ApriltagsCamera", 3, "Network timeout");
						m_network.closeConnection();
					}
				}
			}
		}, 200, 200);
	}

	public void setCameraInfo(double xOffsetInches, double yOffsetInches, double angleOffsetInDegrees) {
		m_cameras.add(new ApriltagsCameraInfo(xOffsetInches, yOffsetInches, angleOffsetInDegrees));
	}

	public void disableCameras(boolean disable) {
		Logger.log("ApriltagsCamera", 1, String.format("DisableCameras = %b", disable));
		m_camerasDisabled = disable;
	}

	/**
	 * Returns true if connected to the camera
	 * 
	 */
	public boolean isConnected() {
		return (m_connected);
	}

	// ! @cond PRIVATE
	public static int[] parseIntegers(String str, int count) {
		int[] args = new int[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Integer.parseInt(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	public static long[] parseLong(String str, int count) {
		long[] args = new long[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Long.parseLong(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	public static double[] parseDouble(String str, int count) {
		double[] args = new double[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Double.parseDouble(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	long m_pingTime = 0;

	public void Ping() {
		m_pingTime = getTimeMs();

		m_network.sendMessage("p");
	}
	// ! @endcond

	/**
	 * Connects to the Raspberry Pi Data server
	 *
	 * @param host - Specifies IP for the host
	 * @param port - Specifies the port (default is 5800)
	 */
	public void connect(String host, int port) {
		m_network = new Network();

		m_startTime = System.currentTimeMillis();

		m_network.connect(this, host, port);

	}

	private void timeSync() {
		long time = getTimeMs();
		if (time > m_syncTime) {
			Logger.log("ApriltagCameras", -1, "TimeSync()");

			m_network.sendMessage(String.format("T1 %d", getTimeMs()));

			m_syncTime = time + k_syncRetry;
		}

	}

	private void processCameraFrame(String args) {
		long a[] = parseLong(args, 10);

		// System.out.println(String.format("%d %s", System.currentTimeMillis(), args));

		if (a != null) {
			int cameraNo = (int) a[0];

			if (cameraNo < m_cameras.size()) {
				ApriltagsCameraInfo info = m_cameras.get(cameraNo);

				m_nextRegions = new ApriltagsCameraRegions(info, (int) a[1], (int) a[2], (int) a[3], (int) a[4],
						(int) a[5],
						(int) a[6],
						a[7], (int) a[8], (int) a[9]);

				int delay = (int) (System.currentTimeMillis() - m_nextRegions.m_captureTime);
				int averageDelay = -1;

				m_averageDelaySum += delay;
				if (++m_averageDelayCount >= m_averageDelayMax) {
					averageDelay = m_averageDelaySum / m_averageDelayCount;

					m_averageDelayCount = 0;
					m_averageDelaySum = 0;
				}

				synchronized (this) {
					if (averageDelay > 0) {
						m_averageDelay = averageDelay;
					}

					if (delay > m_maxDelay) {
						m_maxDelay = delay;
					}
					if (delay < m_minDelay) {
						m_minDelay = delay;
					}

					m_lostFrames = (int) a[5];
				}
			} else {
				Logger.log("ApriltagsCamera", 3, String.format("Camera %d has not been configured", cameraNo));
			}
		}
	}

	/**
	 * Returns the current camera performance stats
	 *
	 */
	public ApriltagsCameraStats getStats() {
		synchronized (this) {
			return (new ApriltagsCameraStats(m_averageDelay, m_maxDelay, m_minDelay, m_lostFrames - m_lastLostFrame,
					System.currentTimeMillis() - m_startTime));
		}
	}

	/**
	 * Clears the camera performance stats
	 *
	 */
	public void clearStats() {
		synchronized (this) {
			m_maxDelay = 0;
			m_minDelay = Integer.MAX_VALUE;
			m_lostFrames = 0;
			m_startTime = System.currentTimeMillis();
		}
	}

	private void processCameraRegion(String args) {
		double a[] = parseDouble(args, 15);

		if ((a != null) && (m_nextRegions != null)) {
			m_nextRegions.addRegion((int) a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11],
					a[12], a[13], a[14]);
		}
	}

	private void processCameraEnd(String args) {
		if (m_nextRegions != null) {
			synchronized (this) {
				m_nextRegions.m_info.m_regions = m_nextRegions;
				m_nextRegions = null;
			}
		}

		timeSync();
	}

	/**
	 * Returns the latest set of camera regions. Note that the data is received from
	 * the camera in a separate thread so calling this twice in a row can generate a
	 * different result. Once you retrieve and instance of ApriltagsCameraRegions
	 * however, you can be assured that it will NOT be modified when a new frame is
	 * received.
	 *
	 */
	public ApriltagsCameraRegions getRegions(int cameraNo) {
		synchronized (this) {
			if (cameraNo < m_cameras.size()) {
				return m_cameras.get(cameraNo).m_regions;
			}
		}

		return null;

	}

	private long getTimeMs() {
		return System.currentTimeMillis();
	}

	private void processTimeSync() {
		Logger.log("ApriltagCamera", -1, "ProcessTimeSync()");

		long time = getTimeMs();

		m_network.sendMessage(String.format("T2 %d", time));
	}

	// ! @cond PRIVATE
	@Override
	public void processData(String data) {
		// Logger.log("ApriltagCamera", 1, String.format("Data: %s", data));

		m_lastMessage = System.currentTimeMillis();

		switch (data.charAt(0)) {
			case 'F':
				processCameraFrame(data.substring(1).trim());
				break;

			case 'R':
				processCameraRegion(data.substring(1).trim());
				break;

			case 'E':
				processCameraEnd(data.substring(1).trim());
				break;

			case 'p':
				Logger.log("ApriltagsCamera", 3, String.format("Ping = %d", getTimeMs() - m_pingTime));
				break;

			case 'T': // sync
				processTimeSync();
				break;

			default:
				Logger.log("ApriltagsCamera", 3, String.format("Invalid command: %s", data));
				break;
		}
	}

	@Override
	public void disconnected() {
		m_connected = false;
	}

	@Override
	public void connected() {
		m_connected = true;

		m_syncTime = getTimeMs() + k_syncFirst;
		m_lastMessage = m_syncTime;
	}
	// ! @endcond

	// int m_frameCount = 0;
	// int m_missingCount = 0;
	// int m_lastFrame = -1;
	// boolean m_logTags = false;

	public void processRegions(SwerveDrivePoseEstimator poseEstimator) {
		if (m_connected) {
			int cameraNo = 0;
			int nProcessed = 0;
			int nFrames = 0;
			for (ApriltagsCameraInfo info : m_cameras) {
				ApriltagsCameraRegions regions = info.m_regions;

				if ((regions != null) && (regions.m_frameNo != info.m_lastFrame)) {
					info.m_lastFrame = regions.m_frameNo;

					// Pose2d currentPose = poseEstimator.getEstimatedPosition();
					// Pose2d currentPose = new Pose2d(0, 0, new Rotation2d(0)); // MUSTFI

					// if (regions.m_regions.size() == 0) Logger.log("ApriltagsCameras", 1, "no
					// regions");

					for (ApriltagsCameraRegion region : regions.m_regions) {
						// Logger.log("ApriltagsCamera", 1, "Calling updatePosition");
						region.updatePosition(cameraNo, poseEstimator, regions.m_captureTime, regions.m_frameNo);
						nProcessed++;
					}
					nFrames++;
				}

				cameraNo++;
			}

			if (m_log && ((nProcessed == 0) && (nFrames > 0))) {
				// If logging is enabled log only the estimated pose
				Pose2d estPos = poseEstimator.getEstimatedPosition();

				m_queue.add(estPos, getTime());
				// Logger.log("ApriltagsCameraLog", 1, ",tag,last yaw,cam yaw,calc yaw,update
				// yaw, est yaw,x,est x,y,est y,adjust");

				Logger.log("ApriltagsCameraLog", 1,
						String.format(",,,,,,%f,,%f,,%f,,,", estPos.getRotation().getDegrees(), estPos.getX(),
								estPos.getY()));
			}
		}
	}

	public static double normalizeAngle(double angle) {
		angle = angle % 360;
		if (angle <= -180) {
			angle += 360;
		} else if (angle >= 180) {
			angle -= 360;
		}
		return angle;
	}

	public void setLogging(boolean log) {
		Logger.log("ApriltagsCamera", 1, String.format("setLogging(%b)", log));

		if (log != m_log) {
			m_log = log;

			if (m_log) {
				m_logTimer.reset();
				m_logTimer.start();

				Logger.setLogFile("ApriltagsCameraLog", "camera", true, false);
				Logger.log("ApriltagsCameraLog", 1,
						",tag,last yaw,cam yaw,calc yaw,update yaw, est yaw,x,est x,y,est y,adjust,frame,dist");
			} else {
				Logger.closeLogFile("ApriltagsCameraLog");
			}
		}
	}

	public void setDashboard(boolean display) {
		Logger.log("ApriltagsCamera", 1, String.format("setDashboard(%b)", display));

		m_dashboard = display;
	}
}
