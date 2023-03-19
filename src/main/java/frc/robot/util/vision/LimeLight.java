
package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase{

    NetworkTable table;

    static Pose2d botPose;

    Pose2d cameraPose;

    static Pose2d targetPose; 
    double[] tagID;


    public LimeLight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void periodic(){

        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        botPose = llresults.fiducialResults.getRobotPose_FieldSpace2D();
        cameraPose = llresults.fiducialResults.getCameraPose_TargetSpace2D();
        targetPose = llresults.fiducialResults.getTargetPose_RobotSpace2D();

        tagID = table.getEntry("tid").getDoubleArray(new double[6]);


        SmartDashboard.getEntry("botpose"); 
        SmartDashboard.getEntry("camerapose_targetspace"); 
        SmartDashboard.getEntry("tid"); 
        SmartDashboard.getEntry("targetpose_robotspace");
       
    }

    public static Pose2d getBotPose(){
        return botPose;
    }

    public static Pose2d getTargetPose(){
        return targetPose;
    }

    public double[] getEntry(String selector) {
        return table.getEntry(selector).getDoubleArray(new double[6]);
    }



}
