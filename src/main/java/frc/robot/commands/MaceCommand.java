package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.mace.Mace;
import frc.robot.subsystems.mace.StoredTrajectories;

public class MaceCommand extends CommandBase {

    private final Mace mSub;
    private Supplier<Boolean> buttonHigh, buttonMid, buttonLow, buttonPick;
    private Supplier<Double> buttonPress;
    public MaceCommand(Mace mSub, Supplier<Boolean> buttonHigh, Supplier<Boolean> buttonMid, Supplier<Boolean> buttonLow,
    Supplier<Double> buttonPress, Supplier<Boolean> buttonPick
    ){
        this.buttonHigh = buttonHigh;
        this.buttonMid = buttonMid;
        this.buttonLow = buttonLow;
        this.buttonPress = buttonPress;
        this.buttonPick = buttonPick;
        this.mSub = mSub;
    addRequirements(mSub);
    }

    @Override
    public void execute() {
    
       // mSub.runMace(buttonLow, buttonPress, StoredTrajectories.LOW_MACE_TRAJECTORY, StoredTrajectories.LOW_BACK_MACE_TRAJECTORY);

       mSub.runMace(buttonHigh, buttonPress, StoredTrajectories.HIGH_MACE_TRAJECTORY, StoredTrajectories.BACK_HIGH_TRAJECTORY);
    }

    @Override
    public void end(boolean interrupted) {
        //mSub.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
