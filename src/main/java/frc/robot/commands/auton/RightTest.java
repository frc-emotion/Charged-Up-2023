package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LevelChargeStation;
import frc.robot.subsystems.SwerveSubsytem;

public class RightTest extends SequentialCommandGroup {
    
    public RightTest (SwerveSubsytem swerveSubsytem, PathPlannerTrajectory path){

        addCommands(
            
        new SequentialCommandGroup(
            SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(5.5), 
        new LockWheels(swerveSubsytem).withTimeout(2)
        ));
    }
}