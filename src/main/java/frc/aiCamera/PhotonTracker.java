package frc.aiCamera;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class PhotonTracker {

    private PhotonCamera camera;
    public PhotonTracker(){
        camera = new PhotonCamera("photonvision");

        
    }

    public void findBestGamePiece(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d camToTarget = target.getBestCameraToTarget();
            List<TargetCorner> corners = target.getDetectedCorners();
            System.out.println("cam to target \n"+camToTarget+"\n corners: \n+"+corners);

        }
    }
 

}
