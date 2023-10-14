package frc.robot.subsystems.mace;

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

    public static final MaceTrajectory HIGH_MACE_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-30))), 
        new ArrayList<Translation2d>() {
            {
            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(170))));     
            }
        }, 
        new MacePoint(0.63, SimConstants.armRadToMeter(Units.degreesToRadians(170))), 
        0.5, 

        0.4);

        public static final MaceTrajectory AUTO_HIGH_MACE_TRAJECTORY = MaceTrajectoryGenerator
        .generateMaceTrajectory(
            new MacePoint(0.6, SimConstants.armRadToMeter(Units.degreesToRadians(-60))), 
            new ArrayList<Translation2d>() {
                {
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));     
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(170))));     
                }
            }, 
            new MacePoint(0.63, SimConstants.armRadToMeter(Units.degreesToRadians(170))), 
            0.5, 
    
            0.4);

        public static final MaceTrajectory BACK_HIGH_TRAJECTORY = MaceTrajectoryGenerator
        .generateMaceTrajectory(
            new MacePoint(0.63, SimConstants.armRadToMeter(Units.degreesToRadians(170))), 
            new ArrayList<Translation2d>() {
                {
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(170))));
                }

            }, 
            new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-30))), 
            0.5, 
    
            0.4);

}
    
   /*  private double bottomAngle = -29.92;

    /*public static final MaceTrajectory START_TO_PICK = MaceTrajectoryGenerator
    .generateMaceTrajectory(
            new MacePoint(0.48, SimConstants.armRadToMeter(Units.degreesToRadians(-68.7))),
            new ArrayList<Translation2d>() {
                {
                    add(new Translation2d(0.48, SimConstants.armRadToMeter(Units.degreesToRadians(-33))));
                    add(new Translation2d(0.12, SimConstants.armRadToMeter(Units.degreesToRadians(-33))));
                }
            },
            new MacePoint(0.12, SimConstants.armRadToMeter(Units.degreesToRadians(-33))),
            0.5,

            0.4);

    public static final MaceTrajectory PICK_TO_START = MaceTrajectoryGenerator
    .generateMaceTrajectory(
            new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-33))),
            new ArrayList<Translation2d>() {
                {
                    add(new Translation2d(0.44, SimConstants.armRadToMeter(Units.degreesToRadians(-33))));
                    add(new Translation2d(0.48, SimConstants.armRadToMeter(Units.degreesToRadians(-55))));
                }
            },
            new MacePoint(0.48, SimConstants.armRadToMeter(Units.degreesToRadians(-55))),
            0.4,

            0.4);*/

   /*  public static final MaceTrajectory HIGH_MACE_TRAJECTORY = MaceTrajectoryGenerator
                    .generateMaceTrajectory(
                    new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-33))),
                    new ArrayList<Translation2d>() {
                        {


                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
                        }
                    },
                    new MacePoint(0.6, SimConstants.armRadToMeter(Units.degreesToRadians(180))),
                    0.5,

                    0.4);
           /*  .generateMaceTrajectory(
                    new MacePoint(0.49, SimConstants.armRadToMeter(Units.degreesToRadians(-45))),
                    new ArrayList<Translation2d>() {
                        {


                            add(new Translation2d(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                            add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
                        }
                    },
                    new MacePoint(0.6, SimConstants.armRadToMeter(Units.degreesToRadians(175))),
                    0.5,

                    0.4);
                    */

   /*  public static final MaceTrajectory MID_MACE_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.49, SimConstants.armRadToMeter(Units.degreesToRadians(-55))),
        new ArrayList<Translation2d>() {
            {


                add(new Translation2d(0.4, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
            }
        },
        new MacePoint(0.218, SimConstants.armRadToMeter(Units.degreesToRadians(176.49))),
        0.5,

        0.4);


    public static final MaceTrajectory LOW_MACE_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-33))),
        new ArrayList<Translation2d>() {
            {


                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                add(new Translation2d(0.2, SimConstants.armRadToMeter(Units.degreesToRadians(180))));
            }
        },
        new MacePoint(0.423, SimConstants.armRadToMeter(Units.degreesToRadians(247))),
        0.5,

        0.4);

    public static final MaceTrajectory MID_BACK_MACE_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.218, SimConstants.armRadToMeter(Units.degreesToRadians(176.49))),
        new ArrayList<Translation2d>() {
            {
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(90))));
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                add(new Translation2d(0.45, SimConstants.armRadToMeter(Units.degreesToRadians(-50))));
            }
        },
        new MacePoint(0.49, SimConstants.armRadToMeter(Units.degreesToRadians(-55))),
        0.5,

        0.4);

    public static final MaceTrajectory LOW_BACK_MACE_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.423, SimConstants.armRadToMeter(Units.degreesToRadians(247))),
        new ArrayList<Translation2d>() {
            {
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(90))));
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                add(new Translation2d(0.45, SimConstants.armRadToMeter(Units.degreesToRadians(-50))));
            }
        },
        new MacePoint(0.49, SimConstants.armRadToMeter(Units.degreesToRadians(-55))),
        0.5,

        0.4);*/

   /* public static final MaceTrajectory BACK_HIGH_TRAJECTORY = MaceTrajectoryGenerator
    .generateMaceTrajectory(
        new MacePoint(0.6, SimConstants.armRadToMeter(Units.degreesToRadians(180))),
        new ArrayList<Translation2d>() {
            {
                add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(120))));
                //add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(0))));
                //add(new Translation2d(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(-33))));
            }
        },
        new MacePoint(0.1, SimConstants.armRadToMeter(Units.degreesToRadians(120))),
        0.5,

        0.4);
        */
//}