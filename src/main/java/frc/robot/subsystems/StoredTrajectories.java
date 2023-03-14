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
    

    public MaceTrajectory generateTrajectory() {

        // 2018 cross scale auto waypoints.
        var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
            Rotation2d.fromDegrees(-180));
        var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
            Rotation2d.fromDegrees(-160));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));
    
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(true);
    
        var trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart,
            interiorWaypoints,
            crossScale,
            config);

        return new MaceTrajectory(trajectory);
      }


    public static final MaceTrajectory TEST_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(Units.inchesToMeters(30), SimConstants.armRadToMeter(Units.degreesToRadians(-75))), 
        new ArrayList<Translation2d>() {
            {
            add(new Translation2d(Units.inchesToMeters(4), SimConstants.armRadToMeter(Units.degreesToRadians(-75))));
            add(new Translation2d(Units.inchesToMeters(10), SimConstants.armRadToMeter(Units.degreesToRadians(90))));
            add(new Translation2d(Units.inchesToMeters(45), SimConstants.armRadToMeter(Units.degreesToRadians(90))));
            }
        }, 
        new MacePoint(Units.inchesToMeters(4), SimConstants.armRadToMeter(Units.degreesToRadians(180))), 
        1, 

        1);

        double testTime(){
            return TEST_TRAJECTORY.totalTime();
        }

        double x = testTime();

}
