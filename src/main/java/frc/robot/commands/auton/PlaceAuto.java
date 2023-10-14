package frc.robot.commands.auton;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.LevelChargeStation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.mace.Mace;
import frc.robot.subsystems.mace.StoredTrajectories;

/**
 * Simple follow trajectory command
 */
public class PlaceAuto extends SequentialCommandGroup {

//     public PlaceAuto(SwerveSubsytem swerveSubsytem,

//             ClawSubsystem claw, Mace mace, ArmSubsystem arm, ElevatorSubsystem e, PathPlannerTrajectory path) {

//         addCommands(
//                 // new ClawAutoCommand(claw, true).withTimeout(4),
//                 // new GetMaceToPose(arm, e, false, true, true).withTimeout(5)
//                 // new RunCommand(() -> e.setElevatorVoltage(0.4), e).withTimeout(2)

//                 // new RunCommand(() -> e.setElevatorVoltage(e.getPIDOutputVolts(0.6)),
//                 // e).withTimeout(2),

//                 new SequentialCommandGroup(
//                         new ParallelCommandGroup(
//                                 new RunMaceTraj(mace, StoredTrajectories.AUTO_HIGH_MACE_TRAJECTORY).withTimeout(4),
//                                 new ClawAutoCommand(claw, true).withTimeout(4)),
//                         new SequentialCommandGroup(
//                                 new ClawAutoCommand(claw, false).withTimeout(1),
//                                 new RunMaceTraj(mace, StoredTrajectories.BACK_HIGH_TRAJECTORY).withTimeout(3.5)),
//                         SwerveController.followTrajectoryCommand(path, true, swerveSubsytem)));
//     }
}