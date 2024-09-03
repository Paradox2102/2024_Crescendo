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

package frc.aiCamera;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.apriltagsCamera.Logger;
import frc.apriltagsCamera.Network;
import frc.robot.ParadoxField;
//import frc.apriltagsCamera.PositionServer;
import frc.robot.PositionTrackerPose;

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
public class AiCamera implements Network.NetworkReceiver {
	private Network m_network = null;
	private boolean m_connected = false;
	private long m_startTime = 0;
	private AiRegions m_regions = null;
	private AiRegions m_nextRegions = null;
	private Object m_lock = new Object();
	public final PositionTrackerPose m_tracker;
	public double m_Robot_x;
	public double m_Robot_y;
	// private Timer m_watchdogTimer = new Timer();

	public class AiRegion {
		public double m_ux;
		public double m_uy;
		public double m_lx;
		public double m_ly;

		public double m_translation_x;
		public double m_translation_y;
		public double m_translation_z;

		public AiRegion(double ux, double uy, double lx, double ly, double translation_x, double translation_y,
				double translation_z) {
			m_ux = ux;
			m_uy = uy;
			m_lx = lx;
			m_ly = ly;

			m_translation_x = translation_x;
			m_translation_y = translation_y;
			m_translation_z = translation_z;
		}
	}

	public class AiRegions {
		public int m_frameNo;
		public int m_width;
		public int m_height;
		public long m_time;
		public ArrayList<AiRegion> m_regions = new ArrayList<AiRegion>();

		public AiRegions(int frameNo, int width, int height) {
			m_frameNo = frameNo;
			m_width = width;
			m_height = height;
			m_time = System.currentTimeMillis() - m_startTime;
		}

		public AiRegion getLargestRegion() {
			AiRegion region = null;
			double largest = 0;

			for (AiRegion r : m_regions) {
				double area = Math.abs((r.m_lx - r.m_ux) * (r.m_ly - r.m_uy));

				if (area > largest) {
					region = r;
					largest = area;
				}
			}

			return region;
		}
	}

	public AiCamera(PositionTrackerPose tracker) {
		m_tracker = tracker;
		// m_watchdogTimer.schedule(new TimerTask() {

		// @Override
		// public void run() {
		// if (m_connected) {
		// Logger.log("ApriltagsCamera", 1, "WatchDog");

		// m_network.sendMessage("k");

		// // if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
		// // Logger.log("ApriltagsCamera", 3, "Network timeout");
		// // m_network.closeConnection();
		// // }
		// }
		// }
		// }, 200, 200);
	}

	public Pose2d FindNotePositions() {
		double m_Robot_x = m_tracker.getPose2d().getTranslation().getX();
		double m_Robot_y = m_tracker.getPose2d().getTranslation().getY();
		double x_distance;
		double y_distance;
		double xr; // xr is x position of note
		double yr; // yr is y position of note
		double alpha;// the degrees the robot needs to turn
		double beta;// robot angle - alpha
		double cx;
		double cy;
		double total_distance; // distance from cam to note
		double distance_from_camera_to_center = -9*.0254; // distance from camera to center of robot in inches converted to meters
		if(m_nextRegions!=null){
			AiRegion largest_region = m_nextRegions.getLargestRegion();
			if(largest_region!=null){
				double transx = largest_region.m_translation_x/39.37;
				double transy = largest_region.m_translation_y/39.37;
				double transz = largest_region.m_translation_z/39.37; // dividing by 39.37 converts inches to meters
				// THESE DISTANCES WERE MADE IN THE EAST, DOWN, NORTH COORDINATE SYSTEM. THEY ARE CONVERTED TO NWU COORDINATE SYSTEM BY CHANGING TRANSZ TO X AND CHANGE TRANS X TO Y
				double x = transz;
				double y = -1*transx;
				double robot_angle = ParadoxField.normalizeAngle(m_tracker.getPose2d().getRotation().getDegrees()-180);
				
				alpha = new Rotation2d(x,y).getDegrees()*-1;//Math.atan2(transx, transz);
				beta = robot_angle - alpha;
				cx = distance_from_camera_to_center*Math.cos(robot_angle);
				cy = distance_from_camera_to_center*Math.sin(robot_angle);
				y_distance = Math.sin(Math.toRadians(beta)) * Math.sqrt(x * x + y * y);
				x_distance = Math.cos(Math.toRadians(beta)) * Math.sqrt(x * x + y * y);
				total_distance = Math.sqrt(x_distance*x_distance+y_distance*y_distance);
				SmartDashboard.putNumber("note distance from robot",total_distance);
				xr = m_Robot_x+x_distance+cx;
				yr = m_Robot_y+y_distance+cy;
				return new Pose2d(xr, yr, Rotation2d.fromDegrees(alpha));
			}
		}
		return null;

																			// robot by adding distance from
																					// camera to robot center
		// x_distance = m_tracker.getPose2d().getRotation().getCos()*transz;
		// TODO: return xr and yr of largest note here

	}

	public AiRegions getRegions() {
		synchronized (m_lock) {
			return m_regions;
		}
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

	// private void timeSync() {
	// long time = getTimeMs();
	// if (time > m_syncTime) {
	// Logger.log("ApriltagCameras", -1, "TimeSync()");

	// m_network.sendMessage(String.format("T1 %d", getTimeMs()));

	// m_syncTime = time + k_syncRetry;
	// }

	// }

	private void processCameraFrame(String args) {
		int a[] = parseIntegers(args, 3);

		// System.out.println(String.format("%d %s", System.currentTimeMillis(), args));

		if (a != null) {
			// Logger.log("AiCamera", 1, String.format("F %d %d %d", a[0], a[1], a[2]));
			m_nextRegions = new AiRegions(a[0], a[1], a[2]);
		}
	}

	private void processCameraRegion(String args) {
		double a[] = parseDouble(args, 7);

		if (a != null) {
			// Logger.log("AiCamera", 1, String.format("R %f %f %f %f", a[0], a[1], a[2],
			// a[3]));
			SmartDashboard.putNumber("AI ul X", a[0]);
			SmartDashboard.putNumber("AI ul Y", a[1]);
			SmartDashboard.putNumber("AI lr X", a[2]);
			SmartDashboard.putNumber("AI lr Y", a[3]);

			SmartDashboard.putNumber("AI trans X", a[4]*.0254);
			SmartDashboard.putNumber("AI trans Y", a[5]*.0254);
			SmartDashboard.putNumber("AI trans Z", a[6]*.0254);

			if (m_nextRegions != null) {
				m_nextRegions.m_regions.add(new AiRegion(a[0], a[1], a[2], a[3], a[4], a[5], a[6]));
			}

		}
	}

	private void processCameraEnd(String args) {
		// Logger.log("AiCamera", 1, "E");
		synchronized (m_lock) {
			m_regions = m_nextRegions;
		}
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
		Logger.log("ApriltagCamera", 1, String.format("Data: %s", data));

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
	}
}
