
package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.mace.Mace;
import frc.robot.subsystems.mace.MaceTrajectory;
import frc.robot.subsystems.mace.StoredTrajectories;

public class RunMaceTraj extends CommandBase {

    private final Mace mace;
    private MaceTrajectory traj;
    private Timer timer;

    public RunMaceTraj(Mace mace, MaceTrajectory traj) {
        timer = new Timer();
        this.mace = mace;
        this.traj = traj;
        addRequirements(mace);
        SmartDashboard.putNumber("Auto timer", timer.get());
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        mace.runAutoTraj(timer.get(), traj);

        SmartDashboard.putNumber("Auto timer", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        // mace.runAutoTraj(timer.get(), traj);
        mace.stopAll();
    }

    @Override
    public boolean isFinished() {
        if (!traj.isActive(timer.get())) {
            timer.stop();
        }
        return false;
    }

}
