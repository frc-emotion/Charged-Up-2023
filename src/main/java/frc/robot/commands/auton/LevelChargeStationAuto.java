package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LevelChargeStation;
import frc.robot.subsystems.SwerveSubsytem;

/**Simple follow trajectory command
 *  */
public class LevelChargeStationAuto extends SequentialCommandGroup {
    
    public LevelChargeStationAuto(SwerveSubsytem swerveSubsytem, PathPlannerTrajectory path){

        addCommands(new SequentialCommandGroup(SwerveController.followTrajectoryCommand(path, true, swerveSubsytem), 
                                                new LevelChargeStation(swerveSubsytem)).withTimeout(6)
    

        );
    }
}