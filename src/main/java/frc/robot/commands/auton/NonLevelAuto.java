package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsytem;

public class NonLevelAuto extends SequentialCommandGroup{
    public NonLevelAuto(SwerveSubsytem swerve) {
        addCommands(new DriveForward(swerve, 0.8).withTimeout(9));
    }
}
