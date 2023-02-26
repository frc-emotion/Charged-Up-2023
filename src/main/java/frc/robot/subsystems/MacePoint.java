package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MacePoint extends Pose2d{
    
    public MacePoint(double h, double theta){
        super(new Translation2d(h, theta), new Rotation2d());
    }

    public MacePoint(Pose2d pose){
        super(pose.getTranslation(), new Rotation2d());
    }

    public double getElevatorHeight(){
        return getX();
    }

    public double getArmAngle(){
        return getY();
    }
}
