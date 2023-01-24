package frc.robot.util.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import frc.robot.Constants.CameraConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.io.IOException;

import java.util.ArrayList; 
import edu.wpi.first.math.Pair;
import java.util.Optional;
import edu.wpi.first.wpilibj.Timer;

public class PoseEstimator extends SubsystemBase {
 
    private final PhotonCamera cam;

    // Physical location of the camera on the robot, relative to the center of the robot. 
    Transform3d robotToCam = new Transform3d(
        new Translation3d(CameraConstants.CAMERA_XAXIS, CameraConstants.CAMERA_YAXIS, CameraConstants.CAMERA_ZAXIS), 
        new Rotation3d(CameraConstants.CAMERA_ROLL, CameraConstants.CAMERA_PITCH, CameraConstants.CAMERA_YAW)
        );

    Transform3d camToRobot = robotToCam.inverse();

    private final RobotPoseEstimator robotPoseEstimator; 
    public AprilTagFieldLayout aprilTagFieldLayout;

    public PoseEstimator(PhotonCamera cam) {

         //Gets Apriltag layout from JSON with IDs and poses.
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile); 
        } catch (IOException e){
            System.out.println(e);
        } 
       
        this.cam = cam; 

        // Assembles the list of cameras & mount locations.
        ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));  

        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);  //FIX test to see what pose strategy is best 

    }

    public Pair<Pose2d, Double> getEstimatedPose() {
        //robotPoseEstimator.setReferencePose(prevEstimatedRobotPose); setup for reference pose strategy only  
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();

        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }   

}