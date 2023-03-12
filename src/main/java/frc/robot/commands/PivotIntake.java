package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class PivotIntake extends CommandBase {
    
    private Intake intake;

    public PivotIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.toggleEndState();
    }
    
    @Override
    public void execute() {
        intake.pivot();
    }

    @Override
    public void end(boolean interrupted) {
        intake.pivotStop();
    }

    @Override
    public boolean isFinished() {
        return intake.checkCurrentSpike();
    }
}