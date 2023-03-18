
package frc.robot.util.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase{

    NetworkTable table;
    private double referenceHeight = 24.125; // height to the middle of the RR tape on the lower bar, in inches
    double[] botPose, cameraPose, tagID, targetPose; 


    public LimeLight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        
    }

    public void periodic(){
        //read values periodically
         botPose = table.getEntry("botpose").getDoubleArray(new double[6]);
         cameraPose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
         tagID = table.getEntry("tid").getDoubleArray(new double[6]);
         targetPose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        //post to smart dashboard periodically
        SmartDashboard.getEntry("botpose"); 
        SmartDashboard.getEntry("camerapose_targetspace"); 
        SmartDashboard.getEntry("tid"); 
        SmartDashboard.getEntry("targetpose_robotspace"); 
       
    }

    public double[] getEntry(String selector) {
        return table.getEntry(selector).getDoubleArray(new double[6]);
    }

/* 

    // LIMELIGHT TURN THEN STRAFE ------------------

    public double getLateral(double tx, double ty){
        return (getHypotenuse(ty)*Math.cos(Math.toRadians(tx)));
    }

    public double getDistance(double tx, double ty){
        return (getHypotenuse(ty)*Math.sin(Math.toRadians(tx)));
    }

    public double getHypotenuse(double ty) {
        return (((getHeight()) / Math.tan(getAngle(Math.toRadians(ty)))) - 0.8); // arbitrary distance away from the target so robot doesnt strafe directly into it?
    }

    // LIMELIGHT SIMULTANEOUS STRAFE/ALIGN ------------------------

    public double getVisionLine(double ty){
        return (getHeight() / Math.tan(Math.toRadians(ty)));
    }

    public double getHypot(double tx, double ty){
        return (getVisionLine(ty)) / Math.cos(Math.toRadians(tx));
    }

    public double getTheta(double tx, double heading){
        return ((Math.PI/2) - Math.abs(heading)  + Math.abs(Math.toRadians(tx)));
    }

    public double getXDist(double tx, double ty, double heading){
        return getHypot(tx, ty) * Math.sin(Math.toRadians(getTheta(tx, heading)));
    }

    public double getYDist(double tx, double ty, double heading){
        return getHypot(tx, ty) * Math.cos(Math.toRadians(getTheta(tx, heading)));
    }

    // GENERAL

    public double getHeight() {
        return Units.inchesToMeters(referenceHeight) - Constants.CameraConstants.CAMERA_ZAXIS; // camera constants need to be changed i think
    }

    public double getAngle(double ty) {
        return ty + Constants.CameraConstants.CAMERA_PITCH;
    }
   
*/

}
