package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmPID;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.auton.GrabberAuton;

public class AutoAbstracted {

    public ClawSubsystem clawSubsystem;
    public ArmSubsystem armSubsystem;
    public ClawSubsystem clawSubSystem;
    public ElevatorSubsystem elevatorSubsystem;

    public AutoAbstracted(SwerveSubsytem swerveSubsytem, ArmSubsystem aSubsystem, ElevatorSubsystem ElevatorSubsystem, ClawSubsystem ClawSubsystem) {
        this.clawSubSystem = ClawSubsystem;
        this.elevatorSubsystem = ElevatorSubsystem;
        this.armSubsystem = aSubsystem;
    }

    public SequentialCommandGroup placeConeMid() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new GrabberAuton(clawSubsystem, Constants.ClawConstants.CONE_INTAKE),
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL)
            ).withTimeout(0.8), // Remove ??

            new ParallelCommandGroup(
                new GrabberAuton(clawSubSystem, Constants.ClawConstants.CONE_INTAKE),
                new ArmPID(armSubsystem, 97),
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL)
            ).withTimeout(3),

            new ParallelCommandGroup(
                new ArmPID(armSubsystem, 97),
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL),
                new GrabberAuton(clawSubSystem, Constants.ClawConstants.CONE_OUTTAKE)
                ).withTimeout(1),

            new ParallelCommandGroup(
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL),
                new ArmPID(armSubsystem, 10)
            ).withTimeout(2)

            //new ArmPID(aSubsystem, 200).withTimeout(3),
            //new ElevatorPID(elevatorSubsystem, 0.04).withTimeout(1.5)
            //SwerveController.followTrajectoryCommand(path, true, swerveSubsytem).withTimeout(5.5)
        
        );
    }


}
