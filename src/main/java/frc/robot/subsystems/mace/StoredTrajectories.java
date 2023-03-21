package frc.robot.subsystems.mace;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.SimConstants;

public class StoredTrajectories {
    
    
    public static final MaceTrajectory TEST_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.04, SimConstants.armRadToMeter(-0.511)), 
        new ArrayList<Translation2d>() {
            {
            add(new Translation2d(0.3, SimConstants.armRadToMeter(0)));
            add(new Translation2d(0.2, SimConstants.armRadToMeter(0)));
            add(new Translation2d(0.1, SimConstants.armRadToMeter(Math.PI/2)));
            add(new Translation2d(0.15, SimConstants.armRadToMeter(Units.degreesToRadians(135))));
            add(new Translation2d(0.3, SimConstants.armRadToMeter(Math.PI)));
            }
        }, 
        new MacePoint(0.5, SimConstants.armRadToMeter(Math.PI)), 
        0.1, 

        0.1);


    public static final MaceTrajectory BACK_TRAJECTORY = MaceTrajectoryGenerator
        .generateMaceTrajectory(
            new MacePoint(0.04, SimConstants.armRadToMeter(-0.511)), 
            new ArrayList<Translation2d>() {
                {
                add(new Translation2d(0.3, SimConstants.armRadToMeter(0)));
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Math.PI/2)));
                add(new Translation2d(0.2, SimConstants.armRadToMeter(Units.degreesToRadians(135))));
                add(new Translation2d(0.3, SimConstants.armRadToMeter(Math.PI)));
                }
            }, 
            new MacePoint(0.5, SimConstants.armRadToMeter(Math.PI)), 
            0.8, 

            0.6);
}
