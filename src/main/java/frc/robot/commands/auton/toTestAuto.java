package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmPID;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.LevelChargeStation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.ClawSubsystem;

public class toTestAuto extends SequentialCommandGroup {
    
    public toTestAuto (SwerveSubsytem swerveSubsytem, ArmSubsystem aSubsystem, ElevatorSubsystem ElevatorSubsystem, ClawSubsystem ClawSubsystem, PathPlannerTrajectory path, Double timeoutTime){
        
        addCommands(

        new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new GrabberAuton(ClawSubsystem, Constants.ClawConstants.CONE_INTAKE),
                    new ElevatorPID(ElevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL)
                ).withTimeout(0.8), // Remove ?? 
    
                new ParallelCommandGroup(
                    new GrabberAuton(ClawSubsystem, Constants.ClawConstants.CONE_INTAKE),
                    new ArmPID(aSubsystem, 97),
                    new ElevatorPID(ElevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL)
                ).withTimeout(3),
    
                new ParallelCommandGroup(
                    new ArmPID(aSubsystem, 97),
                    new ElevatorPID(ElevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL),
                    new GrabberAuton(ClawSubsystem, Constants.ClawConstants.CONE_OUTTAKE)
                    ).withTimeout(1),
    
                new ParallelCommandGroup(
                    new ElevatorPID(ElevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL),
                    new ArmPID(aSubsystem, 10)
                ).withTimeout(2),
    
                new ParallelCommandGroup(
                    //new ArmPID(aSubsystem, 200).withTimeout(3),
                    new ElevatorPID(ElevatorSubsystem, 0.04).withTimeout(1.5),
                    SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime)
                )
            
        ));
    }
}