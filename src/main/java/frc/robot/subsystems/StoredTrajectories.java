package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;
import frc.robot.SimConstants;

public class StoredTrajectories {
    private double bottomAngle = -29.92;

    public static final MaceTrajectory TEST_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-33))), 
        new ArrayList<Translation2d>() {
            {
            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(180))));     
            }
        }, 
        new MacePoint(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(180))), 
        0.5, 

        0.4);

        public static final MaceTrajectory BACK_HIGH_TRAJECTORY = MaceTrajectoryGenerator
        .generateMaceTrajectory(
            new MacePoint(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(180))), 
            new ArrayList<Translation2d>() {
                {
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
                }

            }, 
            new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-33))), 
            0.5, 
    
            0.4);

}
