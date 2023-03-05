package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.SimConstants;

public class StoredTrajectories {
    


    public static final MaceTrajectory TEST_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0, SimConstants.armRadToMeter(Units.degreesToRadians(-75))), 
        new ArrayList<Translation2d>() {
            {
            add(new Translation2d(1, SimConstants.armRadToMeter(Units.degreesToRadians(-75))));
            add(new Translation2d(1, SimConstants.armRadToMeter(Units.degreesToRadians(90))));
            }
        }, 
        new MacePoint(1, SimConstants.armRadToMeter(Units.degreesToRadians(120))), 
        3, 
        2);

}
