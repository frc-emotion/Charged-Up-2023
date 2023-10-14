package frc.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsytem;

/**Simple follow trajectory command
 *  */
public class ExamplePathPlannerCommand extends ParallelCommandGroup {
    
    public ExamplePathPlannerCommand(SwerveSubsytem swerveSubsytem, PathPlannerTrajectory path){

        addCommands(SwerveController.followTrajectoryCommand(path, true, swerveSubsytem));
    }
}