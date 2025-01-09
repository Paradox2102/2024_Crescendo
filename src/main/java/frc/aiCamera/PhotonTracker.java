package frc.aiCamera;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

import org.opencv.core.*;
import org.opencv.photo.CalibrateDebevec;
import org.opencv.calib3d.Calib3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.aiCamera.AiCamera.AiRegions;
import frc.aiCamera.AiCamera.GamePiece;
import frc.apriltagsCamera.Network;
import frc.robot.ParadoxField;
import frc.robot.PositionTrackerPose;

public class PhotonTracker {
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	Calib3d openCV = new Calib3d();
	// private AiRegions[] lastSeenNotes = {};
	private int m_oldFrameNum;
	private Network m_network = null;
	private boolean m_connected = false;
	private int latencyMS = 70; // latency in milliseconds
	private long m_startTime = 0;
	// private AiRegions m_regions = null;
	// private AiRegions m_nextRegions = null;
	private Object m_lock = new Object();
	public final PositionTrackerPose m_tracker;
	public double m_Robot_x;
	public double m_Robot_y;
	public boolean sorting = false;
	private double m_filterDistanceThreshold = 17 * .0254;
	public final long k_timeThresh = 5000;
	private PhotonCamera camera;
	private ArrayList<GamePiece> m_GamePieces = new ArrayList<GamePiece>(); //new gamepieces are gamepieces not saved 
	private static ArrayList<GamePiece> m_oldGamePieces = new ArrayList<GamePiece>(); //old gamepieces are the gamepieces saved in memory for k_timeThresh milliseconds (went through filter distance threshold)

	double radius = 7 * .0254;
	// RESOLUTION: 640x480

	// TODO: CALIBRATE THESE FOR SOLVEPNP TO WORK
	Mat camera_matrix = new Mat(3, 3, CvType.CV_64FC1);
	int rows = 3;
	int columns = 3;
	double[] camera_matrix_data = {775.5212950374786,0.0,266.07398212608825,0.0,771.9111293409104,195.3508972385721,0.0,0.0,1.0};

	// "rows":3,"cols":3,"type":6,"data":[]
	// camera_matrix =
	// {821.0404156441181,0.0,458.4395616969392,0.0,799.2619627088236,179.74026006618715,0.0,0.0,1.0};
	double[] distortion_coefficients_data = {-0.1943467719641159,0.7343284438558819,-4.3240783033692367E-4,-0.0046515275466640724,-0.7164246359491396,0.02632329910813657,-0.028578366736300712,0.027996877307352962};
	MatOfDouble distortion_coefficients = new MatOfDouble(); // [0.11135541962500178,-0.7558352855233885,-0.016663347308976947,0.05014129437440452,1.0290918876431465,-0.008450212518184142,0.03324947495450547,-0.03924149253380953]
	// int radius = 7;
	MatOfPoint2f image_points = new MatOfPoint2f();
	MatOfPoint3f object_points = new MatOfPoint3f(
			new Point3(0, 0, radius),
			new Point3(-radius, 0, 0),
			new Point3(radius, 0, 0),
			new Point3(0, 0, -radius));

	public PhotonTracker(PositionTrackerPose tracker) {
		camera = new PhotonCamera("HD_Camera");
		m_tracker = tracker;
		camera_matrix.put(0, 0, camera_matrix_data);
		distortion_coefficients.put(0, 0, distortion_coefficients_data);
		// image_points.put();
	}



	public class GamePiece {
		public double x1;
		public double y1;
		public double x2; //x1,y1,x2,y2 are bounding box coordinates
		public double y2;
		public String gamePieceName;
		public double xr; //Location in the field
		public double yr;
		public long m_time;

		public GamePiece(double x1, double y1, double x2, double y2,String gamePieceName) {
			this.x1 = x1;
			this.x2=x2;
			this.y1=y1;
			this.y2=y2;
			Pose2d noteLocation = findNoteLocation(x1, y1, x2, y2);
			this.xr = noteLocation.getX(); //automatically sets transX and transY via solvePnP
			this.yr = noteLocation.getY();
			m_time =  System.currentTimeMillis();
			this.gamePieceName = gamePieceName;
		}

	}



	/*public class AiRegions {
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

		public ArrayList<AiRegion> getAllRegionsSorted() {
			Collections.sort(m_regions, new Comparator<AiRegion>() {
				@Override
				public int compare(AiRegion lhs, AiRegion rhs) {
					double area1 = Math.abs((lhs.m_lx - lhs.m_ux) * (lhs.m_ly - lhs.m_uy));
					double area2 = Math.abs((rhs.m_lx - rhs.m_ux) * (rhs.m_ly - rhs.m_uy));
					if (area1 > area2) {
						return -1;
					}
					if (area1 == area2) {
						return 0;
					}
					return 1;
				}
			});
			return m_regions;
		}

		// public AiRegion getLargestRegion() {
		// AiRegion region = null;
		// double largest = 0;

		// for (AiRegion r : m_regions) {
		// double area = Math.abs((r.m_lx - r.m_ux) * (r.m_ly - r.m_uy));

		// if (area > largest) {
		// region = r;
		// largest = area;
		// }
		// }

		// return region;
		// }
	}
*/
	// returns an AiRegion of a given bounding box of a note. Calcuates distance
	public Pose2d findNoteLocation(double x1, double y1, double x2, double y2) {

		image_points = new MatOfPoint2f(); // TODO: initialize image points
		double transx; // NWU coordinate system (transz is replaced by transx and transx is replaced by
						// transy)
		double transy;

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
		double distance_from_camera_to_center = 9 * .0254;
		Mat rvec = new Mat();
		Mat tvec = new Mat();
		image_points = new MatOfPoint2f(
				new Point((x1 + x2) / 2, y1),
				new Point(x1, (y1 + y2) / 2),
				new Point(x2, (y1 + y2) / 2),
				new Point((x1 + x2) / 2, y2));
		Calib3d.solvePnP(object_points, image_points, camera_matrix, distortion_coefficients, rvec, tvec);

		transx = tvec.get(2, 0)[0];
		transy = tvec.get(0, 0)[0];
		// transz = tvec.get(2, 0)[0];
		System.out.println("TRANSX:" + transx);
		System.out.println("TRANSY:" + transy);
		// System.out.println("Z:"+transz);

		// SmartDashboard.put
		// double cameraHeight = 10*.0254; //10 inches to meters
		// System.out.println("target: "+target);
		// System.out.println("pitch: "+target.getPitch());
		// double range =
		// PhotonUtils.calculateDistanceToTargetMeters(cameraHeight,0,Math.toRadians(-20),
		// target.getPitch());

		// Translation2d camToTarget =
		// PhotonUtils.estimateCameraToTargetTranslation(range,
		// Rotation2d.fromDegrees(-target.getYaw()));
		// transx = camToTarget.getX();
		// transy = camToTarget.getY(); //converting meters to inches by dividing by
		// 39.37

		double robot_angle = Math.toRadians(ParadoxField.normalizeAngle(m_tracker.getPose2d().getRotation().getDegrees() - 180));
		double tx = transx-distance_from_camera_to_center;
		double ty = transy;
		double distanceFromRobotCenterToGamePiece = Math.sqrt(tx*tx+ty*ty);
		double xDistanceToNote = Math.cos(robot_angle)*distanceFromRobotCenterToGamePiece;
		double YDistanceToNote = Math.sin(robot_angle)*distanceFromRobotCenterToGamePiece;
		xr = m_Robot_x+xDistanceToNote;
		yr = m_Robot_y+YDistanceToNote;
		return new Pose2d(xr, yr,new Rotation2d()); 
		// return new Pose2d(xr,yr,new Rotation2d());;
		// alpha = new Rotation2d(transx, transy).getDegrees() * -1;// Math.atan2(transx, transz);
		// beta = robot_angle - alpha;
		// cx = distance_from_camera_to_center * Math.cos(robot_angle);
		// cy = distance_from_camera_to_center * Math.sin(robot_angle);
		// y_distance = Math.sin(Math.toRadians(beta)) * Math.sqrt(transx * transx + transy * transy);
		// x_distance = Math.cos(Math.toRadians(beta)) * Math.sqrt(transx * transx + transy * transy);
		// System.out.println("x distance:" + x_distance);
		// System.out.println("y distance: " + y_distance);
		// total_distance = Math.sqrt(x_distance * x_distance + y_distance * y_distance);
		// System.out.println("total Distance:" + total_distance);
		// SmartDashboard.putNumber("note distance from robot (total distance)", total_distance);
		// // SmartDashboard.putNumber("note distance from robot (range)",range);
		// // SmartDashboard.putNumber("Yaw",target.getPitch());

		// xr = m_Robot_x + x_distance + cx;
		// yr = m_Robot_y + y_distance + cy;
		// System.out.println("distance to target: "+range);
		// System.out.println("yaw: "+target.getYaw());

		// transy = tvec.get(0,0)[0];
		// transx = tvec.get(0,2)[0]; //converting to NWU coordinate system
		// transz = tvec.get(0,1)[0];
		
		// Pose2d
		// return null;
	}
	public double calcuateDistanceBetweenTwoPoints(double x1,double y1,double x2, double y2){
		double Xdistance = x2-x1;
		double Ydistance = y2-y1;
		return Math.sqrt(Xdistance*Xdistance+Ydistance*Ydistance);
	}
	public void updateGamePieces() { //updates old gamepieces and new gamepieces 
		
		var result = camera.getLatestResult();
		boolean hasTargets = result.hasTargets();

		if (hasTargets) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			for (PhotonTrackedTarget target : targets) {
				// System.out.println("target:"+target);
				// System.out.println("area:"+target.getArea());
				double x1;
				double y1;
				double x2;
				double y2;
				List<TargetCorner> corners = target.getMinAreaRectCorners();
				// System.out.println("CORNERS size: "+corners.size());
				x1 = corners.get(0).x;
				y1 = corners.get(0).y;
				x2 = corners.get(2).x;
				y2 = corners.get(2).y;
				m_GamePieces.add(new GamePiece(x1, y1, x2, y2, "Note")); //adding all the new gamepieces to the new gamepieces arraylist
			}
		}

			// m_oldGamePieces.removeIf(piece -> ((System.currentTimeMillis() - piece.m_time) > k_timeThresh)); // removing all gamepieces older than k_timeThresh from memory
			for(int i=0;i<m_oldGamePieces.size();i++){ //checking if the new gamepieces pass the distance threshold filter, removing them if they don't
				GamePiece oldGamePiece = m_oldGamePieces.get(i);
				if((System.currentTimeMillis()-oldGamePiece.m_time)>k_timeThresh){
					// System.out.println("removed old note");
					m_oldGamePieces.remove(i);
					i--;
				}
				
				for(int i2=0;i2<m_GamePieces.size();i2++){
					GamePiece newGamePiece = m_GamePieces.get(i2);
					double distanceBetweenGamePieces = calcuateDistanceBetweenTwoPoints(newGamePiece.xr, newGamePiece.yr, oldGamePiece.xr, oldGamePiece.yr);
					if(distanceBetweenGamePieces<m_filterDistanceThreshold){
						m_GamePieces.remove(i2); //removing new gamepiece if its within m_filterDistanceThreshold meters of an old gamepiece.
						i2--;
					}
				}
			}
			for(int i=0;i<m_GamePieces.size();i++){//adding all new gamepieces to memory (old gamepieces) after filter
				m_oldGamePieces.add(m_GamePieces.get(i));
			} 
			m_GamePieces.clear();//clearing all the pieces already added to the old gamepieces

			// sorting = true;
			double robotX = m_tracker.getPose2d().getTranslation().getX();
			double robotY = m_tracker.getPose2d().getTranslation().getY();


			//sorting old gamepieces based on how close they are to the robot with selection sort
			for(int i=0;i<m_oldGamePieces.size()-1;i++){
				GamePiece currentGamePiece = m_oldGamePieces.get(i);
				double localMinimum = calcuateDistanceBetweenTwoPoints(robotX, robotY, currentGamePiece.xr, currentGamePiece.yr);
				int localMinimumIndex = i;
				for(int i2=i+1;i2<m_oldGamePieces.size();i2++){
					GamePiece newGamePiece = m_oldGamePieces.get(i2);
					double currentMinimum = calcuateDistanceBetweenTwoPoints(robotX, robotY, newGamePiece.xr, newGamePiece.yr);
					
					if(currentMinimum<localMinimum){
						localMinimumIndex = i2;
						localMinimum = currentMinimum;
					}
				}
				m_oldGamePieces.set(i, m_oldGamePieces.get(localMinimumIndex)); //swapping current game piece and new gamepiece if new gamepiece is closer to the robot than currentgamepiece
				m_oldGamePieces.set(localMinimumIndex,currentGamePiece);
			}
		
		// sorting = false;
	}

	/*public Vector<Pose2d> FindNotePositions() {

		
		Vector<Pose2d> poses = new Vector<>();
		long cur_time = System.currentTimeMillis();
		int oldpiecesSize = m_old_gamePieces.size();
		
		// System.out.println("old pieces size: "+oldpiecesSize+" newPieces size:
		// "+m_old_gamePieces.size());
		// m_old_gamePieces.removeIf(piece -> (piece.m_translation_x)); //first filter

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
		double distance_from_camera_to_center = -9 * .0254; // distance from camera to center of robot in inches
															// converted to meters
		AiRegions regions = getRegions();
		if (regions != null && m_oldFrameNum < regions.m_frameNo) {
			m_oldFrameNum = regions.m_frameNo;
			// AiRegion largest_region = m_nextRegions.getLargestRegion();

			ArrayList<AiRegion> allRegions = regions.getAllRegionsSorted();
			SmartDashboard.putNumber("Num of AI Regions", allRegions.size());
			for (AiRegion region : allRegions) {
				double transx = region.m_translation_x / 39.37;
				// double transy = region.m_translation_y / 39.37;
				double transz = region.m_translation_z / 39.37; // dividing by 39.37 converts inches to meters
				// THESE DISTANCES WERE MADE IN THE EAST, DOWN, NORTH COORDINATE SYSTEM. THEY
				// ARE CONVERTED TO NWU COORDINATE SYSTEM BY CHANGING TRANSZ TO X AND CHANGE
				// TRANS X TO Y
				double x = transz;
				double y = -1 * transx;
				double robot_angle = ParadoxField.normalizeAngle(m_tracker.getPose2d().getRotation().getDegrees() - 180);

				alpha = new Rotation2d(x, y).getDegrees() * -1;// Math.atan2(transx, transz);
				beta = robot_angle - alpha;
				cx = distance_from_camera_to_center * Math.cos(robot_angle);
				cy = distance_from_camera_to_center * Math.sin(robot_angle);
				y_distance = Math.sin(Math.toRadians(beta)) * Math.sqrt(x * x + y * y);
				x_distance = Math.cos(Math.toRadians(beta)) * Math.sqrt(x * x + y * y);
				System.out.print("x distance:" + x_distance);
				System.out.print("y distance:" + y_distance);
				total_distance = Math.sqrt(x_distance * x_distance + y_distance * y_distance);
				SmartDashboard.putNumber("note distance from robot", total_distance);
				xr = m_Robot_x + x_distance + cx;
				yr = m_Robot_y + y_distance + cy;

			}
			Vector<GamePiece> newGamePieces = new Vector<GamePiece>();
			for (GamePiece old_piece : m_old_gamePieces) {
				boolean passCondition = true; // pass condition to add old note to new notes list. It only passes if
												// it's more than m_filterDistanceThreshold inches of the new notes

				for (Pose2d new_piece : poses) {

					// calcuating distance between old note and new note
					if (new_piece.getTranslation().getDistance(new Translation2d(old_piece.m_translation_x,
							old_piece.m_translation_y)) < m_filterDistanceThreshold) { // checking if old piece is in x
																						// vicinity of new game piece.
						passCondition = false;
						break;
					}
				}
				if (passCondition) {
					newGamePieces.add(old_piece);
				}

			}
			for (Pose2d pose : poses) {
				newGamePieces.add(new GamePiece(pose.getX(), pose.getY(), cur_time));
			}

			m_old_gamePieces = newGamePieces;
			Vector<Pose2d> newPoses = new Vector<>();
			for (GamePiece piece : m_old_gamePieces) {
				newPoses.add(new Pose2d(piece.m_translation_x, piece.m_translation_y, new Rotation2d()));
			}
			return newPoses;
		} else if (!(m_old_gamePieces.isEmpty())) {
			Vector<Pose2d> newPoses = new Vector<>();
			for (GamePiece piece : m_old_gamePieces) {
				newPoses.add(new Pose2d(piece.m_translation_x, piece.m_translation_y, new Rotation2d()));
			}
			return newPoses;
		}
		return null;

		// robot by adding distance from
		// camera to robot center
		// x_distance = m_tracker.getPose2d().getRotation().getCos()*transz;
		// TODO: return xr and yr of largest note here

	}
*/

	public static GamePiece findBestGamePiece() {
		if(m_oldGamePieces.size()>0){
			return m_oldGamePieces.get(0);
		}
		return null;
	}

	public ArrayList<GamePiece> getGamePieces(){
		return m_oldGamePieces;
	}
	public boolean canSeeNote(){
		return m_GamePieces.size()>0;
	}

}
