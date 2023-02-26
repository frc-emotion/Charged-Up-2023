package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class MaceTrajectoryGenerator {

    public static MaceTrajectory generateMaceTrajectory(MacePoint start, List<Translation2d> waypoints, MacePoint end,
            double maxVelocityMPS, double maxAccelMPSS) {

        Trajectory temp = TrajectoryGenerator.generateTrajectory(start, waypoints, end,
                new TrajectoryConfig(maxVelocityMPS, maxAccelMPSS));

        return new MaceTrajectory(temp);
    }

}
