package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class PivotIntake extends CommandBase {
    
    private Intake intake;

    public PivotIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if(RobotContainer.operatorController.getXButtonPressed()){
            intake.setDownPosition();
        }
        if(RobotContainer.operatorController.getYButtonPressed()){
            intake.setUpPosition();
        }
    }
    
    @Override
    public void execute() {
        intake.pivot();
        intake.checkPivotStop();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
