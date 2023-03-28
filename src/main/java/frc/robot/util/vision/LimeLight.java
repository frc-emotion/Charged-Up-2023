
package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase{

    NetworkTable table;

    static Pose2d botPose, cameraPose;
    static Pose3d targetPose;

    double[] tagID;

    public LimeLight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void periodic(){

        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
        botPose = LimelightHelpers.getBotPose2d("limelight");
        targetPose = llresults.fiducialResults.getTargetPose_RobotSpace();

        tagID = table.getEntry("tid").getDoubleArray(new double[6]);

        SmartDashboard.getEntry("botpose"); 
        SmartDashboard.getEntry("targetpose_robotspace");
        SmartDashboard.getEntry("tid"); 
       
    }

    public static Pose2d getBotPose(){
        return botPose;
    }

    public static Pose3d getTargetPose(){
        return targetPose;
    }

    public static double[] getEntry(String selector) {
        return  NetworkTableInstance.getDefault().getTable("limelight").getEntry(selector).getDoubleArray(new double[6]);
    }


    public static double getEntryBasic(String selector) {
        return  NetworkTableInstance.getDefault().getTable("limelight").getEntry(selector).getDouble(0);
    }


}
