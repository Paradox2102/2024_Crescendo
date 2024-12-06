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

public class PhotonTracker{
    Calib3d openCV = new Calib3d();
    // private AiRegions[] lastSeenNotes = {};
	private Vector<GamePiece> m_old_gamePieces = new Vector<GamePiece>(); // gamepieces before filtering
	private int m_oldFrameNum;
	private Network m_network = null;
	private boolean m_connected = false;
	private int latencyMS = 70; //latency in milliseconds
	private long m_startTime = 0;
	private AiRegions m_regions = null;
	private AiRegions m_nextRegions = null;
	private Object m_lock = new Object();
	public final PositionTrackerPose m_tracker;
	public double m_Robot_x;
	public double m_Robot_y;
	private double m_filterDistanceThreshold = 12*.0254;
    public final long k_timeThresh = 3000;
    private PhotonCamera camera;

    int radius = 7;

    //TODO: CALIBRATE THESE FOR SOLVEPNP TO WORK
    Mat camera_matrix;
    MatOfDouble distortion_coefficients;
    MatOfPoint3f object_points;      
    
    public PhotonTracker(PositionTrackerPose tracker){
        camera = new PhotonCamera("HD_Camera");
        m_tracker = tracker;

    }

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
    public class GamePiece {
		public double m_translation_x;
		public double m_translation_y;
		public long m_time;

		public GamePiece(double translation_x, double translation_y, long time) {
			m_translation_x = translation_x;
			m_translation_y = translation_y;
			m_time = time;
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
    
    public Pose2d findNoteLocation(double x1,double y1, double x2,double y2,PhotonTrackedTarget target){//returns an AiRegion of a given bounding box of a note. Calcuates distance
    //    MatOfPoint2f image_points = new MatOfPoint2f(); //TODO: initialize image points
       double transx;
       double transy;
       double transz;
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
		double distance_from_camera_to_center = -9*.0254;
       Mat rvec = new Mat();
       Mat tvec = new Mat();
       Calib3d.solvePnP(object_points, image_points, camera_matrix, distortion_coefficients, rvec, tvec);
       double cameraHeight = 10*.0254; //10 inches to meters
	   System.out.println("target: "+target);
	   System.out.println("pitch: "+target.getPitch());
    //    double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight,0,Math.toRadians(-20), target.getPitch());
       
    //    Translation2d camToTarget = PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-target.getYaw()));
    //    transx = camToTarget.getX();
    //    transy = camToTarget.getY(); //converting meters to inches by dividing by 39.37
       
       double robot_angle = ParadoxField.normalizeAngle(m_tracker.getPose2d().getRotation().getDegrees()-180);
       alpha = new Rotation2d(transx,transy).getDegrees()*-1;//Math.atan2(transx, transz);
       beta = robot_angle - alpha;
       cx = distance_from_camera_to_center*Math.cos(robot_angle);
       cy = distance_from_camera_to_center*Math.sin(robot_angle);
       y_distance = Math.sin(Math.toRadians(beta)) * Math.sqrt(transx * transx + transy * transy);
       x_distance = Math.cos(Math.toRadians(beta)) * Math.sqrt(transx * transx + transy * transy);
       total_distance = Math.sqrt(x_distance*x_distance+y_distance*y_distance);
       SmartDashboard.putNumber("note distance from robot (total distance)",total_distance);
       SmartDashboard.putNumber("note distance from robot (range)",range);
       SmartDashboard.putNumber("Yaw",target.getPitch());

       xr = m_Robot_x+x_distance+cx;
       yr = m_Robot_y+y_distance+cy;
       System.out.println("distance to target: "+range);
       System.out.println("yaw: "+target.getYaw());

    //    transy = tvec.get(0,0)[0];   
    //    transx = tvec.get(0,2)[0]; //converting to NWU coordinate system
    //    transz = tvec.get(0,1)[0];
    //    return new Pose2d(transx, transy,new Rotation2d()); //
    return null;
    } 

    public AiRegions getRegions() {
            var result = camera.getLatestResult();
			boolean hasTargets = result.hasTargets();
            
            if(hasTargets){
                List<PhotonTrackedTarget> targets = result.getTargets();
                for(PhotonTrackedTarget target:targets){
                    double ux; 
                    double uy; 
                    double lx; 
                    double ly;
                    double transx;
                    double transy;//only transx and transy are used since Z is the vertical distance to note which isn't useful
                    List<TargetCorner> corners = target.getDetectedCorners();
                    ux=corners.get(0).x;
                    uy=corners.get(0).y;
                    lx=corners.get(1).x;
                    ly=corners.get(1).y;

                    //TODO: review this
                    // m_nextRegions.m_regions.add(findNoteLocation(ux,uy,lx,ly)); //adding returned AIRegion from findNoteLocatioon to m_regions
                    //not sure whether to return nextRegions or m_regions
                    // not sure what nextRegions is
                    
                }
            }
        return m_regions; //returns detected regions with 3d translations included
	}
	public Vector <Pose2d> FindNotePositions() {
		Vector <Pose2d> poses = new Vector<>();
		long cur_time = System.currentTimeMillis() - m_startTime;
		int oldpiecesSize = m_old_gamePieces.size();
		m_old_gamePieces.removeIf(piece -> ((cur_time-piece.m_time)>k_timeThresh)); //first filter
		// System.out.println("old pieces size: "+oldpiecesSize+" newPieces size: "+m_old_gamePieces.size());
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
		double distance_from_camera_to_center = -9*.0254; // distance from camera to center of robot in inches converted to meters
		AiRegions regions = getRegions();
		if(regions!=null && m_oldFrameNum<regions.m_frameNo){
			m_oldFrameNum = regions.m_frameNo;
			// AiRegion largest_region = m_nextRegions.getLargestRegion();

			ArrayList<AiRegion> allRegions = regions.getAllRegionsSorted(); 
			SmartDashboard.putNumber("Num of AI Regions",allRegions.size());
			for(AiRegion region:allRegions){
				double transx = region.m_translation_x/39.37;
				double transy = region.m_translation_y/39.37;
				double transz = region.m_translation_z/39.37; // dividing by 39.37 converts inches to meters
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
				
			}
			Vector<GamePiece> newGamePieces = new Vector<GamePiece>();
			for(GamePiece old_piece : m_old_gamePieces){
				boolean passCondition = true; //pass condition to add old note to new notes list. It only passes if it's more than m_filterDistanceThreshold inches of the new notes
			
				for(Pose2d new_piece : poses){
	
					//calcuating distance between old note and new note
					if(new_piece.getTranslation().getDistance(new Translation2d(old_piece.m_translation_x,old_piece.m_translation_y))<m_filterDistanceThreshold){ // checking if old piece is in x vicinity of new game piece.
						passCondition = false;
						break;
					}
				}
				if(passCondition){
					newGamePieces.add(old_piece); 
				}
				

			}
			for(Pose2d pose:poses){
				newGamePieces.add(new GamePiece(pose.getX(),pose.getY(),cur_time)); 
			}

			m_old_gamePieces = newGamePieces;
			Vector <Pose2d> newPoses = new Vector<>();
			for(GamePiece piece : m_old_gamePieces){
				newPoses.add(new Pose2d(piece.m_translation_x,piece.m_translation_y,new Rotation2d()));
			}
			return newPoses;
		}
		else if(!(m_old_gamePieces.isEmpty())){
			Vector <Pose2d> newPoses = new Vector<>();
			for(GamePiece piece : m_old_gamePieces){
				newPoses.add(new Pose2d(piece.m_translation_x,piece.m_translation_y,new Rotation2d()));
			}
			return newPoses;
		}
		return null;

																			// robot by adding distance from
																					// camera to robot center
		// x_distance = m_tracker.getPose2d().getRotation().getCos()*transz;
		// TODO: return xr and yr of largest note here

	}






    public Pose2d findBestGamePiece(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            // System.out.println("target: "+target);
            List<TargetCorner> corners = target.getMinAreaRectCorners();
            // System.out.println("number of corners: "+corners.size());
            TargetCorner topLeft = corners.get(0); //top left
            TargetCorner bottomRight = corners.get(2); //top left
            double x1 = topLeft.x; 
            double y1 = topLeft.x; 
            double x2 = bottomRight.x; 
            double y2 = bottomRight.x; 
    
            
            // Transform3d camToTarget = target.getBestCameraToTarget();
            // System.out.println("cam to target \n"+camToTarget+"\n corners: \n+"+corners); //only if photon vision can find note position relative to camera. This probably only works for apriltags.
            Pose2d notePos = findNoteLocation(x1,y1,x2,y2,target);
            return notePos; //returning a po
            
            
            // TargetCorner corner1 = corners.get(0);
            // TargetCorner corner2 = corners.get(1);   //getting all the corners
            // TargetCorner corner3 = corners.get(2);
            // TargetCorner corner4 = corners.get(3);
            

        }
        return null; //return null if no notes can be seen
    }




 

}
