package frc.robot.commands.auton;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmPID;
import frc.robot.commands.ElevatorPID;

public class AutoAbstracted {
    public SwerveSubsytem swerveSubsytem;
    public ClawSubsystem clawSubsystem;
    public ArmSubsystem armSubsystem;
    public ElevatorSubsystem elevatorSubsystem;

    public AutoAbstracted(SwerveSubsytem swerveSubsytem, ArmSubsystem aSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        this.swerveSubsytem = swerveSubsytem;
        this.clawSubsystem = clawSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = aSubsystem;

    } // TODO: Pick up another piece from ground

    public SequentialCommandGroup PlaceConeMid() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new GrabberAuton(clawSubsystem, Constants.ClawConstants.CONE_INTAKE),
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL)
            ).withTimeout(0.8), // Remove ??

            new ParallelCommandGroup(
                new GrabberAuton(clawSubsystem, Constants.ClawConstants.CONE_INTAKE),
                new ArmPID(armSubsystem, 97),
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL)
            ).withTimeout(3),

            new ParallelCommandGroup(
                new ArmPID(armSubsystem, 97),
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL),
                new GrabberAuton(clawSubsystem, Constants.ClawConstants.CONE_OUTTAKE)
                ).withTimeout(1),

            new ParallelCommandGroup(
                new ElevatorPID(elevatorSubsystem, Constants.ElevatorConstants.MIDDLELEVEL),
                new ArmPID(armSubsystem, 10)
            ).withTimeout(2)

        );
    }


}