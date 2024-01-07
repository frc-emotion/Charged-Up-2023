package frc.robot.commands.auton;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.ClawSubsystem;

public class Autonomous extends SequentialCommandGroup {
    

    /*
    public Autonomous (SwerveSubsytem swerveSubsytem, ArmSubsystem aSubsystem, ElevatorSubsystem ElevatorSubsystem, ClawSubsystem ClawSubsystem, PathPlannerTrajectory path, double timeoutTime, boolean place, boolean test){
        
        AutoAbstracted autoCommands = new AutoAbstracted(swerveSubsytem, aSubsystem, ElevatorSubsystem, ClawSubsystem);

        if (test) {
            addCommands(
                new ParallelCommandGroup(
                    autoCommands.TestAuto(),
                    SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime)
                )
            );

        }
        else if (place) {
            addCommands(
                autoCommands.PlaceConeMid(),
                SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime)
                );   
        } else {
            addCommands(
                SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime)
                );
        }

        

        
    }

    public Autonomous (SwerveSubsytem swerveSubsytem, ArmSubsystem aSubsystem, ElevatorSubsystem ElevatorSubsystem, ClawSubsystem ClawSubsystem, PathPlannerTrajectory path, double timeoutTime, boolean place, boolean test, Command AdditionalCommands){
        
        AutoAbstracted autoCommands = new AutoAbstracted(swerveSubsytem, aSubsystem, ElevatorSubsystem, ClawSubsystem);
        
        if (test) {
            addCommands(
                autoCommands.TestAuto(),
                SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime),
                AdditionalCommands
            );
        }

        else if (place) {
            addCommands(
                autoCommands.PlaceConeMid(),
                SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime),
                AdditionalCommands
                );
        } else {
            addCommands(
                SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(timeoutTime),
                AdditionalCommands
                );
        }

        
    }*/

}