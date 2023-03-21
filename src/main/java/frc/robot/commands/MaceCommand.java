package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.mace.Mace;

public class MaceCommand extends CommandBase {

    private final Mace mSub;
    private Supplier<Boolean> button;
    public MaceCommand(Mace mSub, Supplier<Boolean> button) {
        this.button = button;
        this.mSub = mSub;
    addRequirements(mSub);
    }

    @Override
    public void execute() {
        mSub.runMace(button);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
