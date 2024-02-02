package frc.VisionCamera;

import frc.apriltagsCamera.Logger;
import frc.VisionCamera.PiCamera.PiCameraRegion;
import frc.VisionCamera.PiCamera.PiCameraRegions;

public class Camera {
    // @SuppressWarnings("unused")
    private final static String m_ip = "10.21.2.10";

    private final PiCamera m_camera = new PiCamera();

    public class CameraFrame
    {
        PiCameraRegions m_regions = m_camera.getRegions();

        /*
         *
         * Returns true if there is at least one region visible
         */
        public boolean isVisible()
        {
            return (m_regions != null) && (m_regions.getRegionCount() >= 1);
        }

        public PiCameraRegions getRegions() {
            return m_regions;
        }

        /*
         * Returns the horizontal center of the first visible region with respect to the
         * horizontal target position set by the image viewer
         * 
         * NOTE: This function should ONLY be called if isVisible returns true
         */
        public int getTargetCenter() {
                PiCameraRegion region = m_regions.getRegion(0);

            // Logger.Log("Camera", 1, String.format("left=%d, right=%d", region.m_topLeft, region.m_topRight));
    
            return ((region.m_bounds.m_left + region.m_bounds.m_right) / 2) - m_regions.m_targetHorzPos;
        }
        
        public int getCenterLine() {
            return m_regions.m_targetHorzPos;
        } 

        public int getCenterOfTarget() {
            PiCameraRegion region = m_regions.getRegion(0);
            return ((region.m_bounds.m_left + region.m_bounds.m_right) / 2);
        }
    }

    public Camera() {
        // m_camera.connect(m_ip, 5800);
    }

    public void setLight(boolean on) {
        Logger.log("Camera", 2, String.format("SetLight: %s", on ? "on" : "off"));
    }

    public CameraFrame getCurrentFrame()
    {
        return new CameraFrame();
    }
}
