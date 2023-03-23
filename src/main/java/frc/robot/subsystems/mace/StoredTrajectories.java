package frc.robot.subsystems.mace;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.SimConstants;

public class StoredTrajectories {
    private double bottomAngle = -29.92;

    public static final MaceTrajectory TEST_TRAJECTORY = MaceTrajectoryGenerator
            .generateMaceTrajectory(
                    new MacePoint(0.04, SimConstants.armRadToMeter(Units.degreesToRadians(-36))),
                    new ArrayList<Translation2d>() {
                        {
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-30))));
                            add(new Translation2d(0.2, SimConstants.armRadToMeter(Units.degreesToRadians(-20))));
                            add(new Translation2d(0.3, SimConstants.armRadToMeter(Units.degreesToRadians(-10))));
                            add(new Translation2d(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                            add(new Translation2d(0.3, SimConstants.armRadToMeter(Units.degreesToRadians(20))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(90))));
                            add(new Translation2d(0.2, SimConstants.armRadToMeter(Units.degreesToRadians(150))));
                            add(new Translation2d(0.3, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
                        }
                    },
                    new MacePoint(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(180))),
                    0.3,

                    0.3);

    public static final MaceTrajectory BACK_HIGH_TRAJECTORY = MaceTrajectoryGenerator
            .generateMaceTrajectory(
                    new MacePoint(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(180))),
                    new ArrayList<Translation2d>() {
                        {
                            add(new Translation2d(0.3, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
                            add(new Translation2d(0.2, SimConstants.armRadToMeter(Units.degreesToRadians(150))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(90))));
                            add(new Translation2d(0.3, SimConstants.armRadToMeter(Units.degreesToRadians(20))));
                            add(new Translation2d(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                            add(new Translation2d(0.3, SimConstants.armRadToMeter(Units.degreesToRadians(-10))));
                            add(new Translation2d(0.2, SimConstants.armRadToMeter(Units.degreesToRadians(-20))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-30))));
                        }
                    },
                    new MacePoint(0.04, SimConstants.armRadToMeter(Units.degreesToRadians(-35))),
                    0.3,

                    0.3);

    double testTime() {
        return TEST_TRAJECTORY.totalTime();
    }

    double x = testTime();
}