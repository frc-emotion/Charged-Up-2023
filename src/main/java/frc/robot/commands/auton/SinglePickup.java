package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsytem;

public class SinglePickup extends SequentialCommandGroup{
    public SinglePickup(SwerveSubsytem swerveSubsytem, PathPlannerTrajectory path){

        addCommands(SwerveController.followTrajectoryCommand(path, true, swerveSubsytem)
        );
    }
}
