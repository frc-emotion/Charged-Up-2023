
package frc.robot.util.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase{

    private String table; 
    private double refrenceHeight = 24.125; // height to the middle of the RR tape on the lower bar, in inches


    public LimeLight(){
        table = "limelight";
    }

    public double getEntry(String selector) {
        return NetworkTableInstance.getDefault().getTable(table).getEntry(selector).getDouble(0);
    }

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
        return Units.inchesToMeters(refrenceHeight) - Constants.CameraConstants.CAMERA_ZAXIS; // camera constants need to be changed i think
    }

    public double getAngle(double ty) {
        return ty + Constants.CameraConstants.CAMERA_PITCH;
    }
   


}
