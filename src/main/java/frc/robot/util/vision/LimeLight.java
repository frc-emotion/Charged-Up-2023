
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

    public double getLateral(double tx, double ty){
        return (getDistance(ty) * Math.tan(Math.toRadians(tx)));
    }

    public double getDistance(double ty) {
        return ((getHeight()) / Math.tan(getAngle(Math.toRadians(ty))) - 0.1); // arbitrary distance away from the target so robot doesnt strafe directly into it?
    }

    public double getHeight() {
        return Units.inchesToMeters(refrenceHeight) - Constants.CameraConstants.CAMERA_ZAXIS; // camera constants need to be changed i think
    }

    public double getAngle(double ty) {
        return ty + Constants.CameraConstants.CAMERA_PITCH;
    }
   


}
